#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sys/time.h>
#include <driver/gpio.h>

/*
ros2 run tf2_ros static_transform_publisher -0.06743 -0.03446 0.041 0 0 0 1 base_link imu_link
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
socat PTY,link=/tmp/lidar0,raw,echo=0 UDP-LISTEN:9999,reuseaddr,fork
ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=ydlidar_ros2_driver/params/X2.yaml
ros2 launch ./slam_car_cartographer/launch/slam_cartographer.launch.py
rviz2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
*/

// ============ 硬件引脚配置 ============
const int M_PWM[4] = {21, 20, 19, 18};
const int M_IN1[4] = {14, 16, 4, 13};
const int M_IN2[4] = {15, 17, 5, 12};
const int STBY_A = 41;
const int STBY_B = 42;

const int I2C_SDA_FL = 45; // FL 编码器 & MPU6050 -> Wire1
const int I2C_SCL_FL = 46;
const int I2C_SDA_FR = 1; // FR 编码器 -> Wire
const int I2C_SCL_FR = 2;
#define LIDAR_RX 9
#define LIDAR_MCTR 10
const int PWM_FREQ = 10000;
const int PWM_RES = 8;
const int LIDAR_PWM_CH = 4;

// ============ 运动学参数 ============
#define WHEEL_RADIUS 0.030f
#define TRACKWIDTH 0.126f
#define MIN_PWM 130
#define MAX_PWM 255
#define MAX_LINEAR_VEL 3.0f
#define MAX_ANGULAR_VEL 90.0f

// ============ 传感器配置 ============
#define MT6701_I2C_ADDR 0x06
#define REG_ANGLE_MSB 0x03
#define MAX_ANGLE_VALUE 16384.0f
#define ENC_FL 0
#define ENC_RR 1 // 逻辑右轮（现为后右 RR）
#define ENC_COUNT 2

// 编码器方向校正系数（正负号决定该侧速度方向）
// 若前进时建图左转，说明左右符号相反，将一侧取反即可对齐
#define ENC_FL_DIR -1.0f
#define ENC_RR_DIR 1.0f // 默认 -1.0f 修正后轮安装方向相反问题
// ============ 对象与变量 ============
Adafruit_MPU6050 mpu;
char WIFI_SSID[] = "buyaotaoshui";
char WIFI_PASS[] = "buyaotaoshui";
char AGENT_IP[] = "192.168.123.86";

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

HardwareSerial LidarSerial(1);
const IPAddress PC_IP(192, 168, 123, 86);
const uint16_t UDP_PORT = 9999;
#define MAX_PACKET_SIZE 128
uint8_t frameBuf[MAX_PACKET_SIZE];
uint16_t frameIdx = 0;
uint16_t expectedLen = 0;
WiFiUDP udp;
int currentScanHz = 6;

float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
uint32_t last_enc_time = 0;
bool odom_initialized = false;

// ============ 电机控制 ============
void setWheelSigned(int index, float speedRatio)
{
  if (index < 0 || index > 3)
    return;
  float absSpeed = fabs(speedRatio);
  if (absSpeed < 0.01f)
  {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], LOW);
    ledcWrite(index, 0);
    return;
  }
  int duty = (int)(MIN_PWM + (MAX_PWM - MIN_PWM) * absSpeed);
  duty = constrain(duty, MIN_PWM, MAX_PWM);
  if (speedRatio > 0)
  {
    digitalWrite(M_IN1[index], HIGH);
    digitalWrite(M_IN2[index], LOW);
  }
  else
  {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], HIGH);
  }
  ledcWrite(index, duty);
}

void driveDifferential(float vx, float vy, float omega)
{
  vx = constrain(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  omega = constrain(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
  float v_left = vx - omega * (TRACKWIDTH / 2.0f);
  float v_right = vx + omega * (TRACKWIDTH / 2.0f);
  float max_val = fmaxf(fabsf(v_left), fabsf(v_right));
  if (max_val > MAX_LINEAR_VEL)
  {
    float scale = MAX_LINEAR_VEL / max_val;
    v_left *= scale;
    v_right *= scale;
  }
  setWheelSigned(0, v_left / MAX_LINEAR_VEL);
  setWheelSigned(1, v_left / MAX_LINEAR_VEL);
  setWheelSigned(2, v_right / MAX_LINEAR_VEL);
  setWheelSigned(3, v_right / MAX_LINEAR_VEL);
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  driveDifferential(msg->linear.x, msg->linear.y, msg->angular.z);
}

// ============ 编码器读取 ============
float readMT6701Angle(int encoder_idx)
{
  uint8_t msb, lsb;
  int bytesRead = 0;
  if (encoder_idx == ENC_FL)
  {
    Wire1.beginTransmission(MT6701_I2C_ADDR);
    Wire1.write(REG_ANGLE_MSB);
    Wire1.endTransmission(false);
    Wire1.requestFrom(MT6701_I2C_ADDR, (uint8_t)2);
    if (Wire1.available() == 2)
    {
      msb = Wire1.read();
      lsb = Wire1.read();
      bytesRead = 2;
    }
  }
  else if (encoder_idx == ENC_RR) // 👈 改为 ENC_RR
  {
    Wire.beginTransmission(MT6701_I2C_ADDR);
    Wire.write(REG_ANGLE_MSB);
    Wire.endTransmission(false);
    Wire.requestFrom(MT6701_I2C_ADDR, (uint8_t)2);
    if (Wire.available() == 2)
    {
      msb = Wire.read();
      lsb = Wire.read();
      bytesRead = 2;
    } // 注意原代码此处有笔误，应为 Wire.read()，已顺带修正
  }
  else
  {
    return -1.0f;
  }
  if (bytesRead != 2)
    return -1.0f;
  uint16_t rawAngle = (msb << 6) | (lsb & 0x3F);
  return (rawAngle / MAX_ANGLE_VALUE) * 360.0f;
}

float getEncoderSpeed(int enc_idx, float current_angle, uint32_t current_time)
{
  static float last_angle[ENC_COUNT] = {0};
  static uint32_t last_time[ENC_COUNT] = {0};
  if (!odom_initialized)
  {
    last_angle[enc_idx] = current_angle;
    last_time[enc_idx] = current_time;
    return 0;
  }
  uint32_t dt = current_time - last_time[enc_idx];
  if (dt < 5)
    return 0;
  float d_angle = current_angle - last_angle[enc_idx];
  if (d_angle > 180.0f)
    d_angle -= 360.0f;
  if (d_angle < -180.0f)
    d_angle += 360.0f;
  float dt_sec = dt / 1000.0f;
  float angular_vel = d_angle * M_PI / 180.0f / dt_sec;
  float linear_vel = WHEEL_RADIUS * angular_vel;
  last_angle[enc_idx] = current_angle;
  last_time[enc_idx] = current_time;
  return linear_vel;
}

void initEncoders()
{
  Serial.println("🔧 初始化双硬件I2C编码器...");
  // Wire1.begin(I2C_SDA_FL, I2C_SCL_FL);
  Serial.printf("✅ FL (Wire1): SDA=%d, SCL=%d\n", I2C_SDA_FL, I2C_SCL_FL);
  Wire.begin(I2C_SDA_FR, I2C_SCL_FR);
  Serial.printf("✅ FR (Wire):  SDA=%d, SCL=%d\n", I2C_SDA_FR, I2C_SCL_FR);
  delay(50);
  for (int i = 0; i < ENC_COUNT; i++)
  {
    float angle = readMT6701Angle(i);
    if (angle >= 0)
      Serial.printf("  ✅ 编码器%d 角度: %.2f°\n", i, angle);
    else
      Serial.printf("  ❌ 编码器%d 读取失败\n", i);
  }
}

// ============ 里程计更新与发布 ============
void updateOdometry()
{
  uint32_t now = millis();
  if (!odom_initialized)
  {
    last_enc_time = now;
    odom_initialized = true;
    return;
  }
  uint32_t dt_ms = now - last_enc_time;
  if (dt_ms < 20)
    return;
  float dt = dt_ms / 1000.0f;

  float wheel_speed[ENC_COUNT] = {0};
  bool valid[ENC_COUNT] = {false, false};
  for (int i = 0; i < ENC_COUNT; i++)
  {
    float angle = readMT6701Angle(i);
    if (angle >= 0)
    {
      valid[i] = true;
      wheel_speed[i] = getEncoderSpeed(i, angle, now);
    }
  }
  // 应用方向校正系数，确保左右轮前进时速度符号一致
  float v_left = valid[ENC_FL] ? wheel_speed[ENC_FL] * ENC_FL_DIR : 0.0f;
  float v_right = valid[ENC_RR] ? wheel_speed[ENC_RR] * ENC_RR_DIR : 0.0f;
  float v_body = (v_left + v_right) / 2.0f;
  float w_encoder = (v_right - v_left) / TRACKWIDTH;

  odom_theta += w_encoder * dt;
  while (odom_theta > M_PI)
    odom_theta -= 2 * M_PI;
  while (odom_theta < -M_PI)
    odom_theta += 2 * M_PI;
  odom_x += v_body * cosf(odom_theta) * dt;
  odom_y += v_body * sinf(odom_theta) * dt;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  odom_msg.header.stamp.sec = tv.tv_sec;
  odom_msg.header.stamp.nanosec = tv.tv_usec * 1000;
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);
  odom_msg.pose.covariance[0] = 0.01;
  odom_msg.pose.covariance[7] = 0.04;
  odom_msg.pose.covariance[35] = 0.03;
  odom_msg.twist.covariance[0] = 0.01;
  odom_msg.twist.covariance[35] = 0.03;
  odom_msg.twist.twist.linear.x = v_body;
  odom_msg.twist.twist.angular.z = w_encoder;
  (void)rcl_publish(&odom_publisher, &odom_msg, NULL);
  last_enc_time = now;
}

// ============ IMU 发布 ============
void publishImu()
{
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp))
    return; // 读取失败保护

  // 坐标系映射: Sensor(X↑, Y后, Z右) -> ROS(X前, Y左, Z上)
  // Adafruit库已输出 SI 单位 (m/s², rad/s)，无需手动除LSB
  float a_x = -a.acceleration.y;
  float a_y = -a.acceleration.z;
  float a_z = -a.acceleration.x; // 重力向下 => ax≈-9.81 => -(-9.81)=+9.81 (ROS Z向上)

  float g_x = -g.gyro.y;
  float g_y = -g.gyro.z;
  float g_z = g.gyro.x;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  imu_msg.header.stamp.sec = tv.tv_sec;
  imu_msg.header.stamp.nanosec = tv.tv_usec * 1000;
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  imu_msg.linear_acceleration.x = a_x;
  imu_msg.linear_acceleration.y = a_y;
  imu_msg.linear_acceleration.z = a_z;
  imu_msg.angular_velocity.x = g_x;
  imu_msg.angular_velocity.y = g_y;
  imu_msg.angular_velocity.z = g_z;
  imu_msg.orientation_covariance[0] = -1.0;
  (void)rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// ============ 雷达部分 ============
void setLidarSpeed(int hz)
{
  hz = constrain(hz, 5, 8);
  currentScanHz = hz;
  ledcWrite(LIDAR_PWM_CH, map(hz, 5, 8, 255, 128));
}
void startLidar()
{
  setLidarSpeed(currentScanHz);
  delay(100);
  LidarSerial.write(0xA5);
  LidarSerial.write(0x60);
  while (LidarSerial.available())
    LidarSerial.read();
  frameIdx = 0;
  expectedLen = 0;
}
void processLidarByte(uint8_t b)
{
  if (frameIdx >= MAX_PACKET_SIZE)
  {
    frameIdx = 0;
    expectedLen = 0;
    return;
  }
  if (frameIdx == 0)
  {
    if (b == 0xAA)
      frameBuf[frameIdx++] = b;
    return;
  }
  if (frameIdx == 1)
  {
    if (b == 0x55)
      frameBuf[frameIdx++] = b;
    else
      frameIdx = 0;
    return;
  }
  if (frameIdx == 2)
  {
    frameBuf[frameIdx++] = b;
    return;
  }
  if (frameIdx == 3)
  {
    frameBuf[frameIdx++] = b;
    expectedLen = 10 + b * 2;
    if (expectedLen > MAX_PACKET_SIZE)
    {
      frameIdx = 0;
      expectedLen = 0;
    }
    return;
  }
  frameBuf[frameIdx++] = b;
  if (frameIdx >= expectedLen)
  {
    udp.beginPacket(PC_IP, UDP_PORT);
    udp.write(frameBuf, frameIdx);
    udp.endPacket();
    frameIdx = 0;
    expectedLen = 0;
  }
}
void lidarTask(void *pvParameters)
{
  Serial.println("✅ [Core 0] 雷达任务已启动");
  for (;;)
  {
    while (LidarSerial.available())
      processLidarByte(LidarSerial.read());
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ============ Setup ============
void setup()
{
  Serial.begin(115200);
  delay(500);
  for (int i = 0; i < 4; ++i)
  {
    pinMode(M_IN1[i], OUTPUT);
    pinMode(M_IN2[i], OUTPUT);
    digitalWrite(M_IN1[i], LOW);
    digitalWrite(M_IN2[i], LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(M_PWM[i], i);
    ledcWrite(i, 0);
  }
  pinMode(STBY_A, OUTPUT);
  pinMode(STBY_B, OUTPUT);
  digitalWrite(STBY_A, HIGH);
  digitalWrite(STBY_B, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("📡 连接 WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi 连接成功, IP: " + WiFi.localIP().toString());

  configTime(0, 0, "ntp.aliyun.com", "pool.ntp.org");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
    Serial.println("✅ NTP 时间同步成功");

  LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, -1);
  pinMode(LIDAR_MCTR, OUTPUT);
  ledcSetup(LIDAR_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LIDAR_MCTR, LIDAR_PWM_CH);
  udp.begin(UDP_PORT);
  startLidar();
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 2, NULL, 0);

  // ✅ 初始化 I2C 与 MPU6050 (标准库)
  Wire1.begin(I2C_SDA_FL, I2C_SCL_FL);
  Serial.println("初始化 MPU6050 (硬件I2C-1 @ 45/46)...");
  if (!mpu.begin(0x68, &Wire1))
  {
    Serial.println("❌ MPU6050 初始化失败");
  }
  else
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("✅ MPU6050 识别成功");
  }
  initEncoders();

  Serial.println("🔄 正在初始化 micro-ROS...");
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, 8888);
  allocator = rcl_get_default_allocator();
  int retry_count = 0;
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK && retry_count < 10)
  {
    delay(1000);
    retry_count++;
  }
  rclc_node_init_default(&node, "esp32_slam_car", "", &support);
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");
  rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);
  Serial.println("🚀 SLAM 小车系统就绪 (Adafruit MPU6050 + HW I2C Enc)！");
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.reconnect();
    delay(1000);
    return;
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20)
  {
    last_imu_time = millis();
    publishImu();
  }
  static unsigned long last_odom_time = 0;
  if (millis() - last_odom_time >= 20)
  {
    last_odom_time = millis();
    updateOdometry();
  }
}