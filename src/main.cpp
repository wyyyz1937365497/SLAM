/**
 * @file main.cpp
 * @brief ESP32-S3 阿克曼底盘 SLAM 小车 (后轮驱动 + 前轮转向)
 * @version 3.0 (纯 Wire1 手写 MPU6500 驱动版)
 *
 * 核心变更：移除 Adafruit_MPU6050 库，使用纯 Wire1 手写驱动，彻底解决 I2C 冲突
 */

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
#include <rosidl_runtime_c/string_functions.h>
#include <sys/time.h>
#include <driver/gpio.h>
#include <ESP32Servo.h>

// ==================== 硬件引脚配置 ====================
const int M_PWM[2] = {21, 20};
const int M_IN1[2] = {14, 16};
const int M_IN2[2] = {15, 17};
const int STBY = 41;

#define ENC_RL_PULSE_PIN 4
#define ENC_RL_DIR_PIN 5
#define ENC_RR_PULSE_PIN 13
#define ENC_RR_DIR_PIN 12
#define ENC_Z_PIN 2

#define I2C_SDA 45
#define I2C_SCL 46

#define LIDAR_RX 9
#define LIDAR_MCTR 10

const int PWM_FREQ = 10000;
const int PWM_RES = 8;
const int LIDAR_PWM_CH = 4;

// ==================== 阿克曼底盘参数 ====================
// 车轮直径 50mm -> 半径 25mm = 0.025m
#define WHEEL_RADIUS 0.025f
// 前后车轴距离 158mm = 0.158m
#define WHEELBASE 0.158f
// 左右轮距离 116mm = 0.116m
#define TRACKWIDTH 0.116f

#define MIN_PWM 130
#define MAX_PWM 255
#define MAX_LINEAR_VEL 3.0f
#define MAX_STEERING_ANGLE 30.0f
#define MAX_ANGULAR_VEL 60.0f

#define ENC_PULSES_PER_REV 1024
#define ENC_RL_DIR -1.0f
#define ENC_RR_DIR 1.0f

// ==================== 舵机参数配置 ====================
#define STEERING_SERVO_PIN 18
#define SERVO_CENTER_POS 45 // 直行时的绝对角度
#define SERVO_MIN_POS 25    // 绝对角度的最小值（通常是右转极限）
#define SERVO_MAX_POS 65    // 绝对角度的最大值（通常是左转极限）
#define STEERING_RATIO 1.0f
#define STEERING_DIR 1.0f // 转向方向系数：1.0 表示正常，-1.0 表示左右反了
// ==================== 全局对象与变量 ====================
Servo steeringServo;

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
float current_steering_angle = 0.0;

volatile int64_t enc_rl_count = 0;
volatile int64_t enc_rr_count = 0;
volatile unsigned long last_rl_pulse_time = 0;
volatile unsigned long last_rr_pulse_time = 0;

// ==================== 纯 Wire1 手写 MPU6500 驱动 ====================
#define MPU_ADDR 0x68

void writeMPURegister(uint8_t reg, uint8_t value)
{
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

void initMPU6500()
{
  // 1. 唤醒 MPU6500
  writeMPURegister(0x6B, 0x00);
  delay(100);

  // 2. 配置采样率分频 (Sample Rate = 1kHz / (1 + 0))
  writeMPURegister(0x19, 0x00);

  // 3. 配置低通滤波器 (DLPF_CFG = 3, 带宽约 44Hz，适合 SLAM)
  writeMPURegister(0x1A, 0x03);

  // 4. 配置陀螺仪量程 ±500°/s (FS_SEL = 1 -> 0x08)
  writeMPURegister(0x1B, 0x08);

  // 5. 配置加速度计量程 ±4G (AFS_SEL = 1 -> 0x08)
  writeMPURegister(0x1C, 0x08);
}

bool readMPU6500(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B); // ACCEL_XOUT_H
  if (Wire1.endTransmission(false) != 0)
    return false;

  Wire1.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire1.available() < 14)
    return false;

  int16_t raw_ax = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_ay = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_az = (Wire1.read() << 8) | Wire1.read();
  Wire1.read();
  Wire1.read(); // 跳过温度
  int16_t raw_gx = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_gy = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_gz = (Wire1.read() << 8) | Wire1.read();

  // ±4G 灵敏度: 8192 LSB/g, 1g = 9.80665 m/s^2
  *ax = (float)raw_ax / 8192.0 * 9.80665;
  *ay = (float)raw_ay / 8192.0 * 9.80665;
  *az = (float)raw_az / 8192.0 * 9.80665;

  // ±500°/s 灵敏度: 65.5 LSB/(°/s), 1° = PI/180 rad
  *gx = (float)raw_gx / 65.5 * M_PI / 180.0;
  *gy = (float)raw_gy / 65.5 * M_PI / 180.0;
  *gz = (float)raw_gz / 65.5 * M_PI / 180.0;

  return true;
}

// ==================== 中断服务程序 ====================
void IRAM_ATTR encRL_ISR()
{
  bool dirState = digitalRead(ENC_RL_DIR_PIN);
  enc_rl_count += (ENC_RL_DIR > 0) ? (dirState ? 1 : -1) : (dirState ? -1 : 1);
  last_rl_pulse_time = millis();
}

void IRAM_ATTR encRR_ISR()
{
  bool dirState = digitalRead(ENC_RR_DIR_PIN);
  enc_rr_count += (ENC_RR_DIR > 0) ? (dirState ? 1 : -1) : (dirState ? -1 : 1);
  last_rr_pulse_time = millis();
}

// ==================== 电机控制 ====================
void setWheelSigned(int index, float speedRatio)
{
  if (index < 0 || index > 1)
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

void driveAckermann(float vx, float omega)
{
  // 限制输入范围
  vx = constrain(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  omega = constrain(omega, -MAX_ANGULAR_VEL * M_PI / 180.0f, MAX_ANGULAR_VEL * M_PI / 180.0f);

  float steering_angle = 0.0f;

  // ==========================================
  // 1. 计算前轮转角 (修复了左转右转同向的bug)
  // ==========================================
  if (fabs(vx) > 0.01f)
  {
    // 标准阿克曼运动学：角速度 = 线速度 * tan(转角) / 轴距
    // 转角 = atan(角速度 * 轴距 / 线速度)
    steering_angle = atan(omega * WHEELBASE / vx);
  }
  else if (fabs(omega) > 0.01f)
  {
    // 极低速或原地打转时，直接给最大转角
    steering_angle = (omega > 0) ? (MAX_STEERING_ANGLE * M_PI / 180.0f) : -(MAX_STEERING_ANGLE * M_PI / 180.0f);
  }

  // 限制最大转向角
  steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE * M_PI / 180.0f, MAX_STEERING_ANGLE * M_PI / 180.0f);

  // ==========================================
  // 2. 计算后轮差速 (基于刚体运动学)
  // ==========================================
  // 左右轮速度差完全由车体角速度引起
  float left_wheel_speed = vx - (omega * TRACKWIDTH / 2.0f);
  float right_wheel_speed = vx + (omega * TRACKWIDTH / 2.0f);

  // 限制轮速防饱和
  float max_wheel_speed = fmaxf(fabsf(left_wheel_speed), fabsf(right_wheel_speed));
  if (max_wheel_speed > MAX_LINEAR_VEL)
  {
    float scale = MAX_LINEAR_VEL / max_wheel_speed;
    left_wheel_speed *= scale;
    right_wheel_speed *= scale;
  }

  // ==========================================
  // 3. 执行舵机和电机控制
  // ==========================================
  // 映射舵机绝对角度 (加上 STEERING_DIR 以便一键反转方向)
  int servo_angle = SERVO_CENTER_POS + (int)(steering_angle * 180.0f / M_PI * STEERING_RATIO * STEERING_DIR);
  servo_angle = constrain(servo_angle, SERVO_MIN_POS, SERVO_MAX_POS);
  steeringServo.write(servo_angle);

  // 保存当前转向角供里程计使用
  current_steering_angle = steering_angle;

  // 设置后轮速度
  setWheelSigned(0, left_wheel_speed / MAX_LINEAR_VEL);  // 后左轮
  setWheelSigned(1, right_wheel_speed / MAX_LINEAR_VEL); // 后右轮
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  driveAckermann(msg->linear.x, msg->angular.z);
}

// ==================== 编码器读取 ====================
float getEncoderSpeed(int enc_idx, int64_t current_count, uint32_t current_time)
{
  static int64_t last_count[2] = {0, 0};
  static uint32_t last_time[2] = {0, 0};

  if (!odom_initialized)
  {
    last_count[enc_idx] = current_count;
    last_time[enc_idx] = current_time;
    return 0.0f;
  }

  uint32_t dt = current_time - last_time[enc_idx];
  if (dt < 5)
    return 0.0f;

  int64_t count_diff = current_count - last_count[enc_idx];
  float dt_sec = dt / 1000.0f;

  float revolutions = (float)count_diff / ENC_PULSES_PER_REV;
  float linear_vel = revolutions * (2.0f * M_PI * WHEEL_RADIUS) / dt_sec;

  last_count[enc_idx] = current_count;
  last_time[enc_idx] = current_time;
  return linear_vel;
}

void initEncoders()
{
  Serial.println("🔧 初始化脉冲+方向编码器 (后左 + 后右)...");
  pinMode(ENC_RL_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RL_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RL_PULSE_PIN, encRL_ISR, RISING);

  pinMode(ENC_RR_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RR_PULSE_PIN, encRR_ISR, RISING);

  if (ENC_Z_PIN >= 0)
    pinMode(ENC_Z_PIN, INPUT_PULLUP);

  Serial.printf("✅ 后左编码器: PULSE=GPIO%d, DIR=GPIO%d\n", ENC_RL_PULSE_PIN, ENC_RL_DIR_PIN);
  Serial.printf("✅ 后右编码器: PULSE=GPIO%d, DIR=GPIO%d\n", ENC_RR_PULSE_PIN, ENC_RR_DIR_PIN);
}

// ==================== 里程计更新 ====================
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

  noInterrupts();
  int64_t rl_count = enc_rl_count;
  int64_t rr_count = enc_rr_count;
  interrupts();

  float v_left = getEncoderSpeed(0, rl_count, now);
  float v_right = getEncoderSpeed(1, rr_count, now);

  float v_body = (v_left + v_right) / 2.0f;
  float w_encoder = (v_right - v_left) / TRACKWIDTH;

  float cos_steering = cos(current_steering_angle);
  float v_forward = v_body * cos_steering;

  odom_theta += w_encoder * dt;
  while (odom_theta > M_PI)
    odom_theta -= 2 * M_PI;
  while (odom_theta < -M_PI)
    odom_theta += 2 * M_PI;

  odom_x += v_forward * cosf(odom_theta) * dt;
  odom_y += v_forward * sinf(odom_theta) * dt;

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

  odom_msg.twist.twist.linear.x = v_forward;
  odom_msg.twist.twist.angular.z = w_encoder;

  (void)rcl_publish(&odom_publisher, &odom_msg, NULL);
  last_enc_time = now;
}

// ==================== IMU 发布 ====================
void publishImu()
{
  float ax, ay, az, gx, gy, gz;

  // 调用手写驱动读取
  if (!readMPU6500(&ax, &ay, &az, &gx, &gy, &gz))
    return;

  // 坐标系映射: Sensor -> ROS (根据实际安装方向调整)
  float a_x = -ay;
  float a_y = -az;
  float a_z = -ax;
  float g_x = -gy;
  float g_y = -gz;
  float g_z = gx;

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

  // -1 表示无方向估计，让 Cartographer 自己融合
  imu_msg.orientation_covariance[0] = -1.0;

  (void)rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// ==================== 雷达部分 ====================
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

// ==================== Setup ====================
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("🚗 阿克曼底盘 SLAM 小车初始化 (纯手写MPU6500驱动)");
  Serial.println("========================================\n");

  // 初始化电机驱动
  Serial.println("🔧 初始化后轮电机驱动...");
  for (int i = 0; i < 2; ++i)
  {
    pinMode(M_IN1[i], OUTPUT);
    pinMode(M_IN2[i], OUTPUT);
    digitalWrite(M_IN1[i], LOW);
    digitalWrite(M_IN2[i], LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(M_PWM[i], i);
    ledcWrite(i, 0);
  }
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // 初始化舵机
  Serial.println("🔧 初始化前轮转向舵机...");
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(STEERING_SERVO_PIN, 500, 2500);
  steeringServo.write(SERVO_CENTER_POS);
  delay(500);

  // 初始化 WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("📡 连接 WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi 连接成功, IP: " + WiFi.localIP().toString());

  // NTP 时间同步
  configTime(0, 0, "ntp.aliyun.com", "pool.ntp.org");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
    Serial.println("✅ NTP 时间同步成功");

  // ==========================================
  // 核心：纯手写 MPU6500 初始化
  // ==========================================
  Serial.println("🔧 初始化 MPU6500 (Wire1 手写驱动)...");
  Wire1.begin(I2C_SDA, I2C_SCL, 100000);
  initMPU6500();

  // 诊断测试：验证是否成功写入寄存器
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x1B); // 读取陀螺仪配置寄存器
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_ADDR, (uint8_t)1);
  if (Wire1.available() && Wire1.read() == 0x08)
  {
    Serial.println("✅ MPU6500 识别并配置成功 (SLAM优化量程)");
  }
  else
  {
    Serial.println("❌ MPU6500 配置失败");
  }

  // 初始化编码器
  initEncoders();

  // 初始化 micro-ROS
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

  rclc_node_init_default(&node, "esp32_ackermann_slam", "", &support);
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");
  rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);

  // ==========================================
  // 雷达初始化 (移到最后，避免网络栈拥堵)
  // ==========================================
  Serial.println("🔧 初始化雷达...");
  LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, -1);
  pinMode(LIDAR_MCTR, OUTPUT);
  ledcSetup(LIDAR_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LIDAR_MCTR, LIDAR_PWM_CH);
  udp.begin(UDP_PORT);
  startLidar();
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 2, NULL, 0);

  Serial.println("\n========================================");
  Serial.println("🚀 阿克曼 SLAM 小车系统就绪！");
  Serial.println("========================================");
  Serial.printf("📐 轴距: %.3f m\n", WHEELBASE);
  Serial.printf("📐 轮距: %.3f m\n", TRACKWIDTH);
  Serial.printf("⚡ 最大线速度: %.2f m/s\n", MAX_LINEAR_VEL);
  Serial.printf("🔄 最大转向角: %.1f°\n", MAX_STEERING_ANGLE);
  Serial.println("========================================\n");
}

// ==================== Loop ====================
void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.reconnect();
    delay(1000);
    return;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // IMU 发布频率：10ms = 100Hz (Cartographer 强依赖高频 IMU)
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 10)
  {
    last_imu_time = millis();
    publishImu();
  }

  // 里程计发布频率：50Hz (20ms)
  static unsigned long last_odom_time = 0;
  if (millis() - last_odom_time >= 20)
  {
    last_odom_time = millis();
    updateOdometry();
  }
}
