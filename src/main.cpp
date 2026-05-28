#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h> // 新增里程计消息
#include <WiFi.h>
#include <HardwareSerial.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sys/time.h>
#include <math.h>

// ================= 硬件引脚定义 =================
const int M_PWM[4] = {21, 20, 19, 18};
const int M_IN1[4] = {14, 16, 4, 13};
const int M_IN2[4] = {15, 17, 5, 12};
const int STBY_A = 41, STBY_B = 42;
#define LIDAR_RX 9
#define LIDAR_MCTR 10
#define IMU_SDA 6
#define IMU_SCL 7
const int ENC_SDA[4] = {45, 8, 11, 1}; // FL, FR, RL, RR
const int ENC_SCL[4] = {46, 3, 0, 2};
TwoWire I2C_ENC[4] = {TwoWire(0), TwoWire(1), TwoWire(0), TwoWire(1)};

// ================= 系统参数 (已更新) =================
#define WHEEL_RADIUS 0.0325f // 65mm/2 = 0.0325m
#define BASE_WIDTH 0.126f    // 左右轮着地点间距 126mm
#define MIN_PWM 85
#define MAX_PWM 255
#define MAX_LINEAR_VEL 0.5f
#define MAX_ANGULAR_VEL 1.0f
char WIFI_SSID[] = "buyaotaoshui";
char WIFI_PASS[] = "buyaotaoshui";
char AGENT_IP[] = "192.168.123.86";

// ROS 句柄
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t imu_publisher, odom_publisher;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 外设句柄
HardwareSerial LidarSerial(1);
const IPAddress PC_IP(192, 168, 123, 86);
const uint16_t UDP_PORT = 9999;
#define MAX_PACKET_SIZE 128
uint8_t frameBuf[MAX_PACKET_SIZE];
uint16_t frameIdx = 0, expectedLen = 0;
WiFiUDP udp;
int currentScanHz = 6;

// 状态变量
float odom_x = 0, odom_y = 0, odom_theta = 0;
float last_enc_angle[4] = {0, 0, 0, 0};
bool enc_ready[4] = {false, false, false, false};
float gyro_yaw = 0; // MPU6050 无磁力计，改用陀螺仪积分航向

// ================= MPU6050 轻量驱动 =================
#define MPU6050_ADDR 0x68
float acc_x = 0, acc_y = 0, acc_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;

void i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
void i2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  for (int i = 0; i < len; i++)
    if (Wire.available())
      buf[i] = Wire.read();
}

void initMPU6050()
{
  Wire.begin(IMU_SDA, IMU_SCL, 400000);
  i2cWriteReg(MPU6050_ADDR, 0x6B, 0x00); // 唤醒
  i2cWriteReg(MPU6050_ADDR, 0x1A, 0x03); // DLPF 42Hz
  i2cWriteReg(MPU6050_ADDR, 0x1B, 0x00); // 陀螺仪 ±250 dps
  i2cWriteReg(MPU6050_ADDR, 0x1C, 0x00); // 加速度计 ±2g
}

void readMPU6050()
{
  uint8_t buf[14];
  i2cReadReg(MPU6050_ADDR, 0x3B, buf, 14);
  acc_x = (int16_t)(buf[0] << 8 | buf[1]) / 16384.0 * 9.81;
  acc_y = (int16_t)(buf[2] << 8 | buf[3]) / 16384.0 * 9.81;
  acc_z = (int16_t)(buf[4] << 8 | buf[5]) / 16384.0 * 9.81;
  gyro_x = (int16_t)(buf[8] << 8 | buf[9]) / 131.0 * (PI / 180.0);
  gyro_y = (int16_t)(buf[10] << 8 | buf[11]) / 131.0 * (PI / 180.0);
  gyro_z = (int16_t)(buf[12] << 8 | buf[13]) / 131.0 * (PI / 180.0);
}

void getOrientationQuaternion(float &qw, float &qx, float &qy, float &qz)
{
  float roll = atan2(acc_y, acc_z);
  float pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
  gyro_yaw += gyro_z * 0.05f; // 20Hz 积分
  float yaw = gyro_yaw;

  float cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5), sr = sin(roll * 0.5);
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

// ================= MT6701 I2C 读取 =================
float readMT6701(TwoWire &wire)
{
  wire.beginTransmission(0x06);
  wire.write(0x03);
  if (wire.endTransmission(false) != 0)
    return -1.0;
  uint8_t n = wire.requestFrom(0x06, (uint8_t)2);
  if (n != 2)
    return -1.0;
  uint8_t msb = wire.read(), lsb = wire.read();
  return (((msb << 6) | (lsb & 0x3F)) / 16384.0) * 360.0;
}

// ================= 电机控制 =================
void setWheelSigned(int idx, float ratio)
{
  if (idx < 0 || idx > 3)
    return;
  float absR = fabs(ratio);
  if (absR < 0.01f)
  {
    digitalWrite(M_IN1[idx], LOW);
    digitalWrite(M_IN2[idx], LOW);
    ledcWrite(idx, 0);
    return;
  }
  int duty = constrain((int)(MIN_PWM + (MAX_PWM - MIN_PWM) * absR), MIN_PWM, MAX_PWM);
  if (ratio > 0)
  {
    digitalWrite(M_IN1[idx], HIGH);
    digitalWrite(M_IN2[idx], LOW);
  }
  else
  {
    digitalWrite(M_IN1[idx], LOW);
    digitalWrite(M_IN2[idx], HIGH);
  }
  ledcWrite(idx, duty);
}

void driveNormal4WD(float vx, float vy, float omega)
{
  vy = 0;
  vx = constrain(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  omega = constrain(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
  float w_left = vx - (BASE_WIDTH / 2.0f) * omega;
  float w_right = vx + (BASE_WIDTH / 2.0f) * omega;
  float max_w = fmax(fabs(w_left), fabs(w_right));
  if (max_w < 0.001f)
  {
    for (int i = 0; i < 4; i++)
      setWheelSigned(i, 0);
    return;
  }
  float scale = (max_w > MAX_LINEAR_VEL) ? MAX_LINEAR_VEL / max_w : 1.0f;
  w_left *= scale;
  w_right *= scale;
  setWheelSigned(0, w_left);
  setWheelSigned(2, w_left);
  setWheelSigned(1, w_right);
  setWheelSigned(3, w_right);
}

// ================= ROS 回调 =================
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  driveNormal4WD(msg->linear.x, msg->linear.y, msg->angular.z);
}

// ================= 雷达任务 (Core 0) =================
void lidarTask(void *pvParameters)
{
  for (;;)
  {
    while (LidarSerial.available())
    {
      uint8_t b = LidarSerial.read();
      if (frameIdx >= MAX_PACKET_SIZE)
      {
        frameIdx = 0;
        expectedLen = 0;
      }
      if (frameIdx == 0)
      {
        if (b == 0xAA)
          frameBuf[frameIdx++];
        continue;
      }
      if (frameIdx == 1)
      {
        if (b == 0x55)
          frameBuf[frameIdx++];
        else
          frameIdx = 0;
        continue;
      }
      if (frameIdx == 2)
      {
        frameBuf[frameIdx++];
        continue;
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
        continue;
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
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void setLidarSpeed(int hz)
{
  currentScanHz = constrain(hz, 5, 8);
  ledcWrite(4, map(currentScanHz, 5, 8, 255, 128));
}

// ================= Setup =================
void setup()
{
  Serial.begin(115200);
  delay(500);
  // 1. 电机
  for (int i = 0; i < 4; i++)
  {
    pinMode(M_IN1[i], OUTPUT);
    pinMode(M_IN2[i], OUTPUT);
    digitalWrite(M_IN1[i], LOW);
    digitalWrite(M_IN2[i], LOW);
    ledcSetup(i, 10000, 8);
    ledcAttachPin(M_PWM[i], i);
    ledcWrite(i, 0);
  }
  pinMode(STBY_A, OUTPUT);
  pinMode(STBY_B, OUTPUT);
  digitalWrite(STBY_A, HIGH);
  digitalWrite(STBY_B, HIGH);

  // 2. WiFi & NTP
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("📡 连接WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ IP: " + WiFi.localIP().toString());
  configTime(0, 0, "ntp.aliyun.com", "pool.ntp.org");

  // 3. 雷达
  LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, -1);
  pinMode(LIDAR_MCTR, OUTPUT);
  ledcSetup(4, 10000, 8);
  ledcAttachPin(LIDAR_MCTR, 4);
  udp.begin(UDP_PORT);
  setLidarSpeed(currentScanHz);
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 2, NULL, 0);

  // 4. MPU6050
  initMPU6050();
  Serial.println("✅ MPU6050 就绪");

  // 5. 编码器探测 (兼容右后损坏)
  for (int i = 0; i < 4; i++)
  {
    I2C_ENC[i].begin(ENC_SDA[i], ENC_SCL[i], 50000);
    I2C_ENC[i].beginTransmission(0x06);
    enc_ready[i] = (I2C_ENC[i].endTransmission() == 0);
    Serial.printf("📐 编码器 %d %s\n", i, enc_ready[i] ? "在线" : "离线");
  }

  // 6. micro-ROS
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, 8888);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_slam_car", "", &support);

  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");
  rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);

  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "base_link");
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  Serial.println("🚀 系统启动：MPU6050 + 3路编码器里程计 + 激光透传");
}

// ================= Loop =================
void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.reconnect();
    delay(1000);
    return;
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  static unsigned long last_imu = 0, last_odom = 0;
  unsigned long now = millis();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  rcl_time_point_value_t stamp_sec = tv.tv_sec;
  rcl_time_point_value_t stamp_nsec = tv.tv_usec * 1000;

  // 20Hz IMU & 里程计 同步发布
  if (now - last_imu >= 50)
  {
    last_imu = now;
    readMPU6050();

    // 发布 IMU
    imu_msg.header.stamp.sec = stamp_sec;
    imu_msg.header.stamp.nanosec = stamp_nsec;
    float qw, qx, qy, qz;
    getOrientationQuaternion(qw, qx, qy, qz);
    imu_msg.orientation.w = qw;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;
    imu_msg.linear_acceleration.x = acc_x;
    imu_msg.linear_acceleration.y = acc_y;
    imu_msg.linear_acceleration.z = acc_z;
    imu_msg.orientation_covariance[0] = 1e-2;
    imu_msg.orientation_covariance[4] = 1e-2;
    imu_msg.orientation_covariance[8] = 0.1;
    imu_msg.angular_velocity_covariance[0] = 1e-4;
    imu_msg.angular_velocity_covariance[4] = 1e-4;
    imu_msg.angular_velocity_covariance[8] = 1e-4;
    imu_msg.linear_acceleration_covariance[0] = 1e-3;
    imu_msg.linear_acceleration_covariance[4] = 1e-3;
    imu_msg.linear_acceleration_covariance[8] = 1e-3;
    rcl_publish(&imu_publisher, &imu_msg, NULL);

    // 发布 Odom
    float cur_angle[4];
    for (int i = 0; i < 4; i++)
    {
      cur_angle[i] = enc_ready[i] ? readMT6701(I2C_ENC[i]) : last_enc_angle[i];
      if (cur_angle[i] < 0)
        cur_angle[i] = last_enc_angle[i];
    }

    float dL = 0, dR = 0;
    int cntL = 0, cntR = 0;
    // 左侧平均 (FL=0, RL=2)
    for (int i = 0; i < 4; i += 2)
    {
      if (enc_ready[i])
      {
        float d = cur_angle[i] - last_enc_angle[i];
        if (d > 180)
          d -= 360;
        if (d < -180)
          d += 360;
        dL += d;
        cntL++;
        last_enc_angle[i] = cur_angle[i];
      }
    }
    // 右侧平均 (FR=1, RR=3) -> RR损坏自动跳过
    for (int i = 1; i < 4; i += 2)
    {
      if (enc_ready[i])
      {
        float d = cur_angle[i] - last_enc_angle[i];
        if (d > 180)
          d -= 360;
        if (d < -180)
          d += 360;
        dR += d;
        cntR++;
        last_enc_angle[i] = cur_angle[i];
      }
    }
    if (cntL > 0)
      dL /= cntL;
    if (cntR > 0)
      dR /= cntR;

    float circ = 2 * PI * WHEEL_RADIUS;
    float ds_L = dL / 360.0 * circ;
    float ds_R = dR / 360.0 * circ;
    float ds = (ds_L + ds_R) / 2.0;
    float dtheta = (ds_R - ds_L) / BASE_WIDTH;

    odom_x += ds * cos(odom_theta);
    odom_y += ds * sin(odom_theta);
    odom_theta += dtheta;
    while (odom_theta > PI)
      odom_theta -= 2 * PI;
    while (odom_theta < -PI)
      odom_theta += 2 * PI;

    odom_msg.header.stamp.sec = stamp_sec;
    odom_msg.header.stamp.nanosec = stamp_nsec;
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = qx;
    odom_msg.pose.pose.orientation.y = qy;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.twist.twist.linear.x = ds / 0.05f;
    odom_msg.twist.twist.angular.z = dtheta / 0.05f;

    // 2D 里程计协方差 (对角线赋值)
    for (int i = 0; i < 36; i++)
    {
      odom_msg.pose.covariance[i] = 0;
      odom_msg.twist.covariance[i] = 0;
    }
    odom_msg.pose.covariance[0] = 1e-3;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[35] = 1e-3;
    odom_msg.twist.covariance[0] = 1e-3;
    odom_msg.twist.covariance[35] = 1e-3;
    rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
}