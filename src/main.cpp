#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sys/time.h>

// ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
// socat PTY,link=/tmp/lidar0,raw,echo=0 UDP-LISTEN:9999,reuseaddr,fork
// ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=ydlidar_ros2_driver/params/X2.yaml
// ros2 launch ./slam_car_cartographer/launch/slam_cartographer.launch.py
// rviz2
// ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel

const int M_PWM[4] = {21, 20, 19, 18};
const int M_IN1[4] = {14, 16, 4, 13};
const int M_IN2[4] = {15, 17, 5, 12};
const int STBY_A = 41;
const int STBY_B = 42;

const int I2C_SDA = 7;
const int I2C_SCL = 6;

#define LIDAR_RX 9
#define LIDAR_MCTR 10

const int PWM_FREQ = 10000;
const int PWM_RES = 8;
const int LIDAR_PWM_CH = 4;

#define WHEEL_RADIUS 0.03f
#define BASE_WIDTH 0.15f
#define MIN_PWM 85
#define MAX_PWM 255
#define MAX_LINEAR_VEL 0.5f
#define MAX_ANGULAR_VEL 1.0f

char WIFI_SSID[] = "buyaotaoshui";
char WIFI_PASS[] = "buyaotaoshui";
char AGENT_IP[] = "192.168.123.86";

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

HardwareSerial LidarSerial(1);
const IPAddress PC_IP(192, 168, 123, 86);
const uint16_t UDP_PORT = 9999; // 确保与PC端socat一致
#define MAX_PACKET_SIZE 128
uint8_t frameBuf[MAX_PACKET_SIZE];
uint16_t frameIdx = 0;
uint16_t expectedLen = 0;
WiFiUDP udp;
int currentScanHz = 6;

Adafruit_MPU6050 mpu;

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

void driveMecanum(float vx, float vy, float omega)
{
  vx = constrain(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  vy = constrain(vy, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  omega = constrain(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
  float w_FL = vx + vy - (BASE_WIDTH * omega);
  float w_RL = -(vx - vy - (BASE_WIDTH * omega));
  float w_FR = -(vx - vy + (BASE_WIDTH * omega));
  float w_RR = vx + vy + (BASE_WIDTH * omega);
  float wheels[4] = {w_FL, w_RL, w_FR, w_RR};
  float max_val = 0.0;
  for (int i = 0; i < 4; i++)
    if (fabs(wheels[i]) > max_val)
      max_val = fabs(wheels[i]);
  if (max_val < 0.001f)
  {
    for (int i = 0; i < 4; i++)
      setWheelSigned(i, 0);
    return;
  }
  float scale = 1.0f;
  if (max_val > MAX_LINEAR_VEL)
    scale = MAX_LINEAR_VEL / max_val;
  for (int i = 0; i < 4; i++)
    setWheelSigned(i, (wheels[i] * scale) / MAX_LINEAR_VEL);
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  driveMecanum(msg->linear.x, msg->linear.y, msg->angular.z);
}

void setLidarSpeed(int hz)
{
  if (hz < 5)
    hz = 5;
  if (hz > 8)
    hz = 8;
  currentScanHz = hz;
  // 参照驱动说明：M_SCTR 使用 10kHz PWM，且占空比越小转速越快。
  // 这里把 5Hz 作为较慢档，对应接近 100% 占空比；8Hz 作为较快档，对应接近 50% 占空比。
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

// ✅ FreeRTOS 雷达专属任务 (挂在 Core 0)
void lidarTask(void *pvParameters)
{
  Serial.println("✅ [Core 0] 雷达数据搬运任务已启动");
  for (;;)
  {
    while (LidarSerial.available())
    {
      processLidarByte(LidarSerial.read());
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // 1ms 延迟，喂看门狗并让出CPU
  }
}

void publishImu()
{
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp))
    return;

  // ✅ 修改：获取系统绝对时间（与 PC 时间同步）
  struct timeval tv;
  gettimeofday(&tv, NULL);
  imu_msg.header.stamp.sec = tv.tv_sec;
  imu_msg.header.stamp.nanosec = tv.tv_usec * 1000;

  // 设置 frame_id
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // 线性加速度 (m/s^2) - 根据你的安装姿态映射
  imu_msg.linear_acceleration.x = a.acceleration.y;
  imu_msg.linear_acceleration.y = -a.acceleration.z;
  imu_msg.linear_acceleration.z = a.acceleration.x;

  // 角速度 - 根据你的安装姿态映射
  imu_msg.angular_velocity.x = g.gyro.y;
  imu_msg.angular_velocity.y = -g.gyro.z;
  imu_msg.angular_velocity.z = g.gyro.x;

  // 姿态不可用
  imu_msg.orientation_covariance[0] = -1.0;

  (void)rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // 1. 初始化电机 (必须最早初始化，防止引脚悬空导致乱跑)
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

  // // 在 WiFi.begin() 之前加入以下扫描代码
  // Serial.println("🔍 开始扫描附近 WiFi...");
  // int n = WiFi.scanNetworks();
  // Serial.printf("扫描完成，发现 %d 个网络:\n", n);
  // for (int i = 0; i < n; ++i)
  // {
  //   Serial.printf("%d: %s (%d dBm) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "开放" : "加密");
  // }
  // 如果列表里没有 HONOR300，说明就是手机热点频段/信道的问题

  // 2. 连接 WiFi (雷达和 micro-ROS 都依赖网络)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("📡 连接 WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi 连接成功, IP: " + WiFi.localIP().toString());

  // ✅ 新增：配置 NTP 服务器进行网络对时
  Serial.println("🔄 正在同步 NTP 时间...");
  configTime(0, 0, "ntp.aliyun.com", "pool.ntp.org"); // 使用阿里云和公共NTP服务器
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("❌ NTP 同步失败");
  }
  else
  {
    Serial.println("✅ NTP 时间同步成功");
  }

  // ⚠️ 关键顺序调整：先启动雷达和UDP，确保PC端能立刻握手！
  // 3. 初始化雷达引脚及串口
  LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, -1);
  pinMode(LIDAR_MCTR, OUTPUT);
  ledcSetup(LIDAR_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LIDAR_MCTR, LIDAR_PWM_CH);

  // 4. 启动 UDP 透传
  udp.begin(UDP_PORT); // 绑定本地 9999
  startLidar();
  Serial.println("✅ 雷达已启动，数据正在透传...");

  // ✅ 5. 创建雷达专属高优先级任务 (绑定到 Core 0)
  // 这样即使在下面初始化 MPU6050 和 micro-ROS 耗时几秒，雷达数据也不会丢
  xTaskCreatePinnedToCore(
      lidarTask,   // 任务函数
      "LidarTask", // 任务名
      4096,        // 堆栈大小
      NULL,        // 参数
      2,           // 优先级 (高于默认的1)
      NULL,        // 任务句柄
      0            // 核心编号 (Core 0)
  );

  // 6. 初始化 I2C 与 MPU6050 (耗时操作，但此时雷达任务在 Core 0 已经在跑，不受影响)
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin(0x68, &Wire))
  {
    Serial.println("❌ 找不到 MPU6050！");
  }
  else
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("✅ MPU6050 初始化完成");
  }

  // 7. 初始化 micro-ROS (最耗时的操作，放在最后)
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, 8888);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_slam_car", "", &support);
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);

  Serial.println("🚀 SLAM 小车系统就绪！");
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("⚠️ WiFi断开，尝试重连...");
    WiFi.reconnect();
    delay(1000);
    return;
  }

  // 雷达数据已在 Core 0 独立运行，主循环安心处理 ROS 和 IMU
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20)
  {
    last_imu_time = millis();
    publishImu();
  }
}
