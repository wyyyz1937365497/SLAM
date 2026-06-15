/**
@file main.cpp
@brief ESP32-S3 阿克曼底盘 SLAM 小车 (后轮驱动 + 前轮转向 + 独立双PID闭环电机控制)
@version 3.3 (集成最新智能死区与独立双PID算法，彻底解决停机抖动与收敛慢问题)
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
#define WHEEL_RADIUS 0.025f
#define WHEELBASE 0.158f
#define TRACKWIDTH 0.116f
#define ENC_PULSES_PER_REV 2048
#define MOTOR_ENCODER_RATIO 2.0f

// 编码器方向系数 (配合调试好的 ISR 使用)
#define ENC_RL_DIR -1.0f
#define ENC_RR_DIR 1.0f

#define MAX_LINEAR_VEL 2.45f
#define MAX_STEERING_ANGLE 30.0f
#define MAX_ANGULAR_VEL 60.0f

// ==================== 舵机参数配置 ====================
#define STEERING_SERVO_PIN 18
#define SERVO_CENTER_POS 46
#define SERVO_MIN_POS 25
#define SERVO_MAX_POS 65
#define STEERING_RATIO 1.0f
#define STEERING_DIR 1.0f

// ==================== 独立 PID 参数配置 (左轮 L / 右轮 R) ====================
// 默认值设为平滑保守值，可根据实际测试通过代码或串口动态调整
float Kp_L = 156.0, Ki_L = 15.0, Kd_L = 0.0;
float Kp_R = 156.0, Ki_R = 15.0, Kd_R = 0.0;

#define MAX_INTEGRAL_PWM 80.0 // 稍微放宽积分限幅，防止过早饱和
#define MIN_START_PWM 35      // 🚨 核心修改：从 46 降至 35，减少横跳概率

// ==================== 全局对象与变量 ====================
Servo steeringServo;

char WIFI_SSID[] = "wyyyz";
char WIFI_PASS[] = "12345678";
char AGENT_IP[] = "10.42.0.1";

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
const IPAddress PC_IP(10, 42, 0, 1);
const uint16_t UDP_PORT = 9999;
#define MAX_PACKET_SIZE 128
uint8_t frameBuf[MAX_PACKET_SIZE];
uint16_t frameIdx = 0;
uint16_t expectedLen = 0;
WiFiUDP udp;
int currentScanHz = 6;

// 里程计变量
float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
bool odom_initialized = false;
float current_steering_angle = 0.0;

// 编码器计数
volatile int64_t enc_rl_count = 0;
volatile int64_t enc_rr_count = 0;

// 【PID移植】PID 控制全局变量
float target_v_left = 0.0;
float target_v_right = 0.0;
float integral[2] = {0.0, 0.0};
float prev_error[2] = {0.0, 0.0};
int64_t last_enc_count[2] = {0, 0};
unsigned long last_pid_time = 0;

// 【调试用】记录最近的 ROS 指令
float last_cmd_vx = 0.0;
float last_cmd_omega = 0.0;

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
  writeMPURegister(0x6B, 0x00);
  delay(100);
  writeMPURegister(0x19, 0x00);
  writeMPURegister(0x1A, 0x03);
  writeMPURegister(0x1B, 0x08);
  writeMPURegister(0x1C, 0x08);
}

bool readMPU6500(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B);
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

  *ax = (float)raw_ax / 8192.0 * 9.80665;
  *ay = (float)raw_ay / 8192.0 * 9.80665;
  *az = (float)raw_az / 8192.0 * 9.80665;
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
}

void IRAM_ATTR encRR_ISR()
{
  bool dirState = digitalRead(ENC_RR_DIR_PIN);
  int direction = (ENC_RR_DIR > 0) ? (dirState ? 1 : -1) : (dirState ? -1 : 1);
  enc_rr_count -= direction; // 【核心修正】原来是 +=，改为 -= 实现取反
}

// ==================== 电机控制 ====================
void setMotorDirectionAndSpeed(int index, int speed_pwm)
{
  speed_pwm = constrain(speed_pwm, -255, 255);
  if (abs(speed_pwm) < 5)
  {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], LOW);
    ledcWrite(index, 0);
    return;
  }

  if (speed_pwm > 0)
  {
    digitalWrite(M_IN1[index], HIGH);
    digitalWrite(M_IN2[index], LOW);
  }
  else
  {
    digitalWrite(M_IN1[index], LOW);
    digitalWrite(M_IN2[index], HIGH);
  }
  ledcWrite(index, abs(speed_pwm));
}

// ==================== 运动学解算 ====================
void driveAckermann(float vx, float omega)
{
  last_cmd_vx = vx;
  last_cmd_omega = omega;

  vx = constrain(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  omega = constrain(omega, -MAX_ANGULAR_VEL * M_PI / 180.0f, MAX_ANGULAR_VEL * M_PI / 180.0f);

  float steering_angle = 0.0f;
  if (fabs(vx) > 0.01f)
  {
    steering_angle = atan(omega * WHEELBASE / vx);
  }
  else if (fabs(omega) > 0.01f)
  {
    steering_angle = (omega > 0) ? (MAX_STEERING_ANGLE * M_PI / 180.0f) : -(MAX_STEERING_ANGLE * M_PI / 180.0f);
  }
  steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE * M_PI / 180.0f, MAX_STEERING_ANGLE * M_PI / 180.0f);

  float left_wheel_speed = vx - (omega * TRACKWIDTH / 2.0f);
  float right_wheel_speed = vx + (omega * TRACKWIDTH / 2.0f);

  float max_wheel_speed = fmaxf(fabsf(left_wheel_speed), fabsf(right_wheel_speed));
  if (max_wheel_speed > MAX_LINEAR_VEL)
  {
    float scale = MAX_LINEAR_VEL / max_wheel_speed;
    left_wheel_speed *= scale;
    right_wheel_speed *= scale;
  }

  int servo_angle = SERVO_CENTER_POS + (int)(steering_angle * 180.0f / M_PI * STEERING_RATIO * STEERING_DIR);
  servo_angle = constrain(servo_angle, SERVO_MIN_POS, SERVO_MAX_POS);
  steeringServo.write(servo_angle);
  current_steering_angle = steering_angle;

  target_v_left = left_wheel_speed;
  target_v_right = right_wheel_speed;

  // 🚨 核心修复：停止时不仅清零积分，还清零误差历史，防止积分冻结导致停机抖动
  if (fabs(vx) < 0.01f && fabs(omega) < 0.01f)
  {
    integral[0] = 0.0f;
    integral[1] = 0.0f;
    prev_error[0] = 0.0f;
    prev_error[1] = 0.0f;
  }
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  driveAckermann(msg->linear.x, msg->angular.z);
}

// ==================== 统一的 PID 控制与里程计更新 (20ms) ====================
void updateControlAndOdometry()
{
  unsigned long now = millis();
  if (now - last_pid_time >= 20)
  {
    float dt = (now - last_pid_time) / 1000.0f;
    last_pid_time = now;

    if (!odom_initialized)
    {
      last_enc_count[0] = enc_rl_count;
      last_enc_count[1] = enc_rr_count;
      odom_initialized = true;
      return;
    }

    noInterrupts();
    int64_t current_counts[2] = {enc_rl_count, enc_rr_count};
    interrupts();

    float dir_factors[2] = {ENC_RL_DIR, ENC_RR_DIR};
    float actual_v[2];
    float errors[2] = {0.0, 0.0};
    float derivatives[2] = {0.0, 0.0};
    int speed_pwms[2] = {0, 0};

    // 【调试用】降频打印计数器：每 25 次 (500ms) 打印一次
    static int debug_cnt = 0;
    debug_cnt++;
    bool print_debug = (debug_cnt >= 25);
    if (print_debug)
      debug_cnt = 0;

    for (int i = 0; i < 2; i++)
    {
      int64_t delta = current_counts[i] - last_enc_count[i];
      last_enc_count[i] = current_counts[i];

      // 1. 计算实际速度 (m/s)
      float wheel_revolutions = (float)delta / (ENC_PULSES_PER_REV * MOTOR_ENCODER_RATIO);
      float raw_speed = wheel_revolutions * (2.0f * M_PI * WHEEL_RADIUS) / dt;
      actual_v[i] = raw_speed * dir_factors[i];

      // 2. PID 闭环计算
      float target_v = (i == 0) ? target_v_left : target_v_right;
      float error = target_v - actual_v[i];
      errors[i] = error;

      // 动态选择当前轮子的 PID 参数
      float kp = (i == 0) ? Kp_L : Kp_R;
      float ki = (i == 0) ? Ki_L : Ki_R;
      float kd = (i == 0) ? Kd_L : Kd_R;

      // 积分抗饱和 (使用动态 ki)
      if (error * integral[i] >= 0 && abs(kp * error + ki * integral[i]) < 240)
      {
        integral[i] += error * dt;
      }
      if (abs(ki * integral[i]) > MAX_INTEGRAL_PWM)
      {
        integral[i] = (integral[i] > 0 ? 1 : -1) * MAX_INTEGRAL_PWM / ki;
      }

      float derivative = (error - prev_error[i]) / dt;
      derivatives[i] = derivative;
      prev_error[i] = error;

      // 计算 PID 输出
      int speed_pwm = (int)(kp * error + ki * integral[i] + kd * derivative);

      // 🚨 核心修复：智能死区补偿
      // 仅在误差较大（启动或大幅变速阶段）时，才强制提升 PWM 克服静摩擦。
      // 当速度接近目标时，允许 PWM 平滑衰减到 0，彻底消除稳态抖动。
      if (abs(error) > 0.1f)
      {
        if (abs(speed_pwm) >= 5 && abs(speed_pwm) < MIN_START_PWM)
        {
          speed_pwm = (speed_pwm > 0) ? MIN_START_PWM : -MIN_START_PWM;
        }
      }
      speed_pwms[i] = speed_pwm;

      // 3. 执行电机控制
      setMotorDirectionAndSpeed(i, speed_pwm);
    }

    // 【调试输出】打印核心 PID 变量，帮助排查抖动
    if (print_debug)
    {
      Serial.printf("\n[ROS_CMD] vx: %.2f, omega: %.2f | Target -> L: %.2f, R: %.2f\n",
                    last_cmd_vx, last_cmd_omega, target_v_left, target_v_right);
      Serial.printf("[PID_DBG] L: act=%.2f, err=%.2f, int=%.2f, der=%.2f, pwm=%4d\n",
                    actual_v[0], errors[0], integral[0], derivatives[0], speed_pwms[0]);
      Serial.printf("[PID_DBG] R: act=%.2f, err=%.2f, int=%.2f, der=%.2f, pwm=%4d\n",
                    actual_v[1], errors[1], integral[1], derivatives[1], speed_pwms[1]);
      Serial.println("--------------------------------------------------");
    }

    // 4. 里程计更新 (复用刚刚计算出的 actual_v，保证数据严格同步)
    float v_left = actual_v[0];
    float v_right = actual_v[1];

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
  }
}

// ==================== IMU 发布 ====================
void publishImu()
{
  float ax, ay, az, gx, gy, gz;
  if (!readMPU6500(&ax, &ay, &az, &gx, &gy, &gz))
    return;

  float a_x = ay;
  float a_y = az;
  float a_z = ax;
  float g_x = gy;
  float g_y = gz;
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
  Serial.println("🚗 阿克曼底盘 SLAM 小车 (独立双PID + 智能死区)");
  Serial.println("========================================\n");

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

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(STEERING_SERVO_PIN, 500, 2500);
  steeringServo.write(SERVO_CENTER_POS);
  delay(500);

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

  Wire1.begin(I2C_SDA, I2C_SCL, 100000);
  initMPU6500();
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x1B);
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_ADDR, (uint8_t)1);
  if (Wire1.available() && Wire1.read() == 0x08)
  {
    Serial.println("✅ MPU6500 识别并配置成功");
  }

  pinMode(ENC_RL_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RL_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RL_PULSE_PIN, encRL_ISR, RISING);

  pinMode(ENC_RR_PULSE_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_RR_PULSE_PIN, encRR_ISR, RISING);

  if (ENC_Z_PIN >= 0)
    pinMode(ENC_Z_PIN, INPUT_PULLUP);
  Serial.println("✅ 编码器初始化完成");

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

  LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX, -1);
  pinMode(LIDAR_MCTR, OUTPUT);
  ledcSetup(LIDAR_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LIDAR_MCTR, LIDAR_PWM_CH);
  udp.begin(UDP_PORT);
  startLidar();
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 2, NULL, 0);

  last_pid_time = millis();
  Serial.println("\n🚀 系统就绪！独立双PID与智能死区已激活。");
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

  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 10)
  {
    last_imu_time = millis();
    publishImu();
  }

  updateControlAndOdometry();
}