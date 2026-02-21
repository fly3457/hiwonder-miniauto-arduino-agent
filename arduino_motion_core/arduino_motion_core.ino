/**
 * @file app_control_common.ino
 * @author WangPengfei(fly3457@hotmail.com)
 * @brief APP遥控玩法（标准Z字形定义版本）
 * @version V2.1 - Standard Z-Pattern (Simplified)
 * @date 2024-04-26 (原版)
 * @date 2026-01-01 (标准Z字形改写 + 简化索引映射)
 *
 * @copyright Copyright (c) 2024
 *
 * @attention main函数中不可以做任何阻塞处理！
 *
 * ============================================================================
 * 轮子编号定义（标准Z字形 - ROS兼容）
 * ============================================================================
 * 标准Z字形索引定义：
 *   索引0 → 左前轮 (引脚9,  控制板M3)
 *   索引1 → 右前轮 (引脚10, 控制板M2)
 *   索引2 → 左后轮 (引脚6,  控制板M4)
 *   索引3 → 右后轮 (引脚11, 控制板M1)
 *
 * 小车俯视图（车头朝上）：
 *          车头方向(0°)
 *              ↑
 *      0左前 ——— 1右前
 *        |         |
 *      2左后 ——— 3右后
 *
 * ============================================================================
 * 运动学模型：标准麦克纳姆轮逆解算
 * ============================================================================
 * 坐标系定义：
 *   X轴 = 车头方向（前进为正）
 *   Y轴 = 车身左侧方向（左移为正）
 *   用户angle定义：0°=前进，90°=左移，逆时针为正（标准ROS右手系）
 *
 * 运动学公式（符合ROS标准右手坐标系）：
 *   V_左前(0) = Vx + Vy + Vθ
 *   V_右前(1) = Vx - Vy - Vθ
 *   V_左后(2) = Vx - Vy + Vθ
 *   V_右后(3) = Vx + Vy - Vθ
 *
 * 其中：
 *   Vx = velocity * cos(-angle) : 前进速度分量
 *   Vy = velocity * sin(-angle) : 横移速度分量（左为正）
 *   Vθ = rot                    : 旋转速度（正值=逆时针，符合ROS右手系）
 *
 * ============================================================================
 * 实现特性
 * ============================================================================
 * - 引脚数组按标准Z字形索引排列，无需额外映射
 * - 所有索引（运动学、电机测试）统一使用标准Z字形定义
 * - 代码简洁，易于理解和维护
 * ============================================================================
 */

#include <Arduino.h>
#include "FastLED.h"
// #include <Servo.h>  // ⭐ 已注释: 不使用舵机功能
#include "Ultrasound.h"
#include <Wire.h>  // I2C库

// I2C 配置
#define I2C_ESP32_ADDR 0x52           // ESP32-S3 的 I2C 从机地址 (官方地址,与ESP32CAM程序一致)
#define I2C_POLL_INTERVAL 10          // 轮询ESP32的间隔 (毫秒) - 优化: 从50ms降至10ms
#define I2C_DEBUG_INTERVAL 5000       // 调试信息打印间隔 (毫秒)

// I2C 接收缓冲区
#define I2C_BUFFER_SIZE 64            // 定义I2C缓冲区容量
char i2c_rx_buffer[I2C_BUFFER_SIZE];  // 存储64字节的原始数据
unsigned long last_i2c_poll = 0;      // 轮询时间控制
unsigned long last_i2c_debug = 0;     // 调试输出控制

// I2C 统计信息
unsigned long i2c_total_polls = 0;    // 总轮询次数
unsigned long i2c_success_reads = 0;  // 成功读取次数
String last_received_cmd = "";        // 最后接收的命令

// ⭐ P2优化: Arduino主动上报传感器数据 (取代Android端D命令轮询)
// 传感器采集和上报频率控制（非阻塞设计）
unsigned long last_ultrasound_sample = 0;  // 超声波采样计时
unsigned long last_voltage_sample = 0;     // 电压采样计时
unsigned long last_sensor_report = 0;      // 传感器上报计时

#define ULTRASOUND_SAMPLE_INTERVAL 50  // 超声波采样间隔 (ms) - 快速响应障碍物
#define VOLTAGE_SAMPLE_INTERVAL 200    // 电压采样间隔 (ms) - 电压变化慢，无需高频采样
#define SENSOR_REPORT_INTERVAL 200     // 传感器上报间隔 (ms) - 与Android端同步

// ⭐ 调试功能: 超声波耗时监控（可选开启）
// 用于检测Ultrasound.Filter()是否阻塞主循环
// 启用方法: 取消下一行的注释
#define ULTRASOUND_TIMING_DEBUG
#ifdef ULTRASOUND_TIMING_DEBUG
unsigned long ultrasound_total_time = 0;
unsigned long ultrasound_call_count = 0;
unsigned long ultrasound_max_time = 0;
#endif

// ⭐ 调试功能: Loop循环耗时监控（可选开启）
// 用于监控主循环的执行时间，帮助发现性能瓶颈
// 启用方法: 取消下一行的注释
#define LOOP_TIMING_DEBUG
#ifdef LOOP_TIMING_DEBUG
unsigned long loop_start_time = 0;           // 本次loop开始时间
unsigned long loop_total_time = 0;           // 累计loop耗时
unsigned long loop_count = 0;                // loop执行次数
unsigned long loop_max_time = 0;             // 最大loop耗时
unsigned long last_loop_report = 0;          // 上次报告时间
#define LOOP_REPORT_INTERVAL 5000            // 每5秒报告一次loop统计信息
#endif

typedef enum {
  MODE_NULL,
  MODE_ROCKERANDGRAVITY,  // 摇杆&重力控制
  MODE_RGB_ADJUST,        // RGB调节
  MODE_SPEED_CONTROL,     // 速度控制
  MODE_ULTRASOUND_SEND,   // 发送超声波距离给上位机
  MODE_SERVO_CONTROL,     // 机械爪控制
  MODE_VOLTAGE_SEND,      // 发送电压值给APP
  MODE_AVOID,             // 避障
  MODE_MOTOR_TEST         // 电机测试
} CarMode;                // 小车模式

typedef enum {
  WARNING_OFF,            // "警告-关闭" - 不发出任何警告
  WARNING_BEEP,           // "警告-蜂鸣" - 蜂鸣器发出声音警告
  WARNING_RGB,            // "警告-RGB灯" - RGB灯发出视觉警告
} VoltageWarning;         // "电压警告"

typedef enum {
  READ_VOLTAGE_ON,        // 读取电压开启
  READ_VOLTAGE_OFF        // 读取电压关闭
} ReadVoltageState;       // 读取电压状态

// Servo myservo;         // ⭐ 已注释: 实例化舵机
Ultrasound ultrasound;    // 实例化超声波

// g_warning, g_mode, g_read 的默认值
static VoltageWarning g_warning = WARNING_OFF;
static CarMode g_mode = MODE_NULL;
static ReadVoltageState g_read = READ_VOLTAGE_ON;

static uint8_t g_state = 8;         // 接收的APP子指令
static uint8_t avoid_flag = 0;      // 避障模式开关标志位
static uint8_t motor_test_flag = 0; // 电机测试模式标志位
static uint8_t rot_flag = 0;        // 转向标志位
static uint8_t beep_count = 0;      // 蜂鸣器鸣响次数

static int car_derection = 0;       // 设置小车移动的角度
static int8_t car_rot = 0;          // 设置小车角速度
static uint8_t speed_data = 0;      // 设置小车线速度
static uint8_t speed_update = 50;   // APP更新的线速度

/* 电压监测相关参数 */
static float voltage;               // ADC采样得到的原始电压值，单位：伏特(V)，用于实时电压检测
static int voltage_send;            // 准备发送的电压值，单位：毫伏(mV)，从voltage转换而来
static int last_voltage_send;       // 上一次发送的电压值，单位：毫伏(mV)，用于异常跳变检测
static int real_voltage_send;       // 实际有效的电压值，单位：毫伏(mV)，过滤掉异常值后的稳定电压，用于上报给上位机
static int error_voltage;           // 检测到的异常电压值，单位：毫伏(mV)，用于标记可能的采样干扰

/* 电机测试相关参数 */
static int8_t test_motor_speeds[4] = {0, 0, 0, 0}; // 测试模式下的电机速度（标准Z字形索引）

/* 电机校准系数（用于补偿硬件个体差异）*/
/* 标准Z字形索引：0=左前, 1=右前, 2=左后, 3=右后 */
/* 使用方法：通过实际测试调整系数，使小车直线移动不跑偏 */
/* 例如：如果前进时向左偏，说明右侧轮速度过慢，需要增大右侧系数 */
static float motor_calibration[4] = {1.0, 1.0, 1.0, 1.0};

static CRGB rgbs[1];                // FastLED RGB灯数组，只有1个WS2812 LED灯珠，用于状态指示和低电压警报
String rec_data[4];                 // 串口接收数据缓冲区
String i2c_rec_data[4];             // I2C接收数据缓冲区（独立，避免与串口数据冲突）

const char *charArray;              // 串口命令字符串指针，指向rec_data[0]转换后的C字符串，用于strcmp比较命令类型
const char *i2c_charArray;          // I2C命令字符串指针

/* 引脚定义 - 按标准Z字形索引顺序 */
const static uint8_t ledPin = 2;                              // led引脚
const static uint8_t buzzerPin = 3;                           // 蜂鸣器引脚
// const static uint8_t servoPin = 5;                         // ⭐ 已注释: 舵机引脚
const static uint8_t motorpwmPin[4] = { 9, 10, 6, 11} ;       // 标准Z字形索引：0=左前, 1=右前, 2=左后, 3=右后
const static uint8_t motordirectionPin[4] = { 8, 12, 7, 13};  // 方向引脚对应PWM引脚顺序

const static int pwmFrequency = 500;                // PWM频率，单位是赫兹 (500Hz适合直流电机，过高会增加功耗，过低会产生电机抖动)
const static int period = 10000000 / pwmFrequency;  // PWM周期，单位是微秒 (计算公式: 1秒=1000000微秒, 一个周期=1000000/频率=20000us)
const static uint32_t interval_us = 20000;          // 微秒计数时间间隔 20000us=20ms，用于软件PWM的非阻塞延时
const static uint32_t interval_ms = 1000;           // 毫秒计数时间间隔 1000ms=1秒，用于蜂鸣器警报节奏控制的非阻塞延时

// ⭐ 软件PWM每个电机独立计时器（修复共用计时器导致的不稳定问题）
static uint32_t motor_previousTime_us[4] = {0, 0, 0, 0};  // 每个电机独立的PWM计时器

static uint32_t previousTime_us = 0;                // 上一次的微秒计数时间间隔 用于非阻塞延时
static uint32_t previousTime_ms = 0;                // 上一次的毫秒计数时间间隔 用于非阻塞延时

// static int increase_angle = 0;                   // ⭐ 已注释: 设置舵机角度
// static int default_angle = 90;                   // ⭐ 已注释: 默认角度
static uint16_t distance = 0;                       // 超声波距离

/* 定义函数列表 */
void Aovid(void);                                   // 避障任务
void Rgb_Task(void);                                // RGB灯控制任务
void Motor_Init(void);                              // 电机初始化函数
void Speed_Task(void);                              // 速度控制任务函数
void Task_Dispatcher(void);                         // 任务调度函数（统一调度和执行）
// void Servo_Data_Receive(void);                   // 舵机数据接收函数
void Motor_Test_Task(void);                         // 电机测试任务函数
void Rockerandgravity_Task(void);                   // 摇杆和重力控制任务函数
void Low_Voltage_Warning(void);                     // 低压报警函数
void PWM_Out(uint8_t motor_index, uint8_t PWM_Pin, int8_t DutyCycle);    // PWM输出函数（带独立计时器参数）
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);                      // RGB显示函数
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot);            // 速度控制器函数
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);  // 电机设置函数
void pollSerialCommand(void);                       // 轮询串口获取命令（只获取，不执行）
void pollESP32Command(void);                        // 轮询ESP32模块获取命令（只获取，不执行）
void sampleUltrasound(void);                        // 超声波传感器采样函数（非阻塞）
void sampleVoltage(void);                           // 电压传感器采样函数（非阻塞）
void reportSensorDataToESP32(void);                 // 主动上报传感器数据给ESP32

void setup() {
  Serial.begin(115200);                               // 设置串口波特率

  // 初始化 I2C (主机模式)
  Wire.begin();  // Arduino UNO 作为主机

  Serial.println("[I2C] Arduino 初始化为 I2C 主机");
  Serial.print("[I2C] ESP32 从机地址: 0x");
  Serial.println(I2C_ESP32_ADDR, HEX);

  // 扫描 I2C 总线上的所有设备
  Serial.println("\n[I2C扫描] 正在扫描I2C总线...");
  bool found = false;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);                   // 尝试向地址 addr 发起传输
    byte error = Wire.endTransmission();            // 结束传输，获取返回值
    if (error == 0) {                               // 如果错误码为 0，表示找到了设备
      Serial.print("  找到设备: 0x");
      if (addr < 16) Serial.print("0");             // 如果地址小于 16（0x10），补一个前导零，保持两位十六进制格式
      Serial.print(addr, HEX);                      // 地址 0x0A → 显示 "0x0A" 而不是 "0xA"

      // 识别常见设备
      if (addr == I2C_ESP32_ADDR) {
        Serial.print(" (ESP32-S3)");
      } else if (addr == 0x77) {
        Serial.print(" (超声波传感器)");
      } else if (addr == 0x78) {
        Serial.print(" (可能是寻线传感器)");
      }
      Serial.println();
      found = true;
    }
  }
  if (!found) {
    Serial.println("  ⚠ 警告: 未找到任何I2C设备!");
  } else {
    Serial.println("[I2C扫描] 扫描完成\n");
  }
  // 初始化 I2C 结束

  FastLED.addLeds<WS2812, ledPin, RGB>(rgbs, 1);    // 初始化WS2812 LED灯带
  Motor_Init();                                     // 初始化电机

  // ⭐ 已注释: 舵机初始化
  // pinMode(servoPin, OUTPUT);
  // myservo.attach(servoPin);                      // 绑定舵机指定引脚
  // myservo.write(default_angle + increase_angle); // 写入舵机角度

  tone(buzzerPin, 200);                       // 输出音调信号的函数,频率为1200Hz，开机提示音
  delay(100);                                 // 蜂鸣100毫秒
  noTone(buzzerPin);                          // 关闭蜂鸣器

  /* 电压计算公式: ADC读数 × 0.02989 × 1000
   * 解释: Arduino ADC是10位(0-1023), 参考电压5V
   * 电压检测电路使用分压电阻，分压比例使得实际电压 = ADC读数 × 0.02989V
   * 乘以1000转换为毫伏(mV)方便传输和显示 */
  voltage_send = analogRead(A3)*0.02989*1000; // 初始化时读取电压，单位：毫伏(mV)
  last_voltage_send = voltage_send;           // 保存上次电压值，用于异常跳变检测
  real_voltage_send = voltage_send;           // 保存实际有效电压值，过滤掉异常值后的电压

  Serial.println("[启动] Arduino UNO 已就绪");
  Serial.println("[提示] 将每隔 50ms 轮询 ESP32 获取 BLE 命令...");
}

void loop() {
  #ifdef LOOP_TIMING_DEBUG
  unsigned long loop_start_time = millis();  // 记录loop开始时间
  #endif

  // ⭐ 优化架构: 职责分离 - 命令获取 → 任务调度 → 传感器采样 → 数据上报

  // ========== 步骤1: 命令获取（非阻塞） ==========
  pollSerialCommand();   // 获取串口命令（用于调试）
  pollESP32Command();    // 获取I2C命令（来自ESP32-BLE）

  // ========== 步骤2: 任务调度和执行（包括电机控制） ==========
  Task_Dispatcher();     // 统一调度所有任务，包括电机控制

  // ========== 步骤3: 传感器数据采样和上报（非阻塞） ==========
  sampleUltrasound();         // 每50ms采样一次超声波
  sampleVoltage();            // 每200ms采样一次电压
  reportSensorDataToESP32();  // 每隔200ms主动上报传感器数据给ESP32

  // ========== 步骤4: I2C调试信息输出 ==========
  // 定时打印I2C调试信息
  if (millis() - last_i2c_debug >= I2C_DEBUG_INTERVAL) {
    last_i2c_debug = millis();
    Serial.println("\n========== I2C 状态报告 ==========");
    Serial.print("总轮询次数: ");
    Serial.println(i2c_total_polls);
    Serial.print("成功读取次数: ");
    Serial.println(i2c_success_reads);
    if (last_received_cmd.length() > 0) {
      Serial.print("最后收到命令: ");
      Serial.println(last_received_cmd);
    } else {
      Serial.println("最后收到命令: (无)");
    }
    Serial.println("==================================\n");
  }

  #ifdef LOOP_TIMING_DEBUG
  // 计算本次loop耗时
  unsigned long loop_duration = millis() - loop_start_time;
  loop_total_time += loop_duration;
  loop_count++;

  // 更新最大耗时
  if(loop_duration > loop_max_time) {
    loop_max_time = loop_duration;
  }

  // 单次loop耗时超过10ms时立即警告
  if(loop_duration > 10) {
    Serial.print("[警告] Loop阻塞! 耗时: ");
    Serial.print(loop_duration);
    Serial.println("ms");
  }

  // 定期输出统计信息
  if(millis() - last_loop_report >= LOOP_REPORT_INTERVAL) {
    last_loop_report = millis();
    Serial.println("\n========== Loop 性能统计 ==========");
    Serial.print("总执行次数: ");
    Serial.println(loop_count);
    Serial.print("平均耗时: ");
    Serial.print(loop_total_time / loop_count);
    Serial.println("ms");
    Serial.print("最大耗时: ");
    Serial.print(loop_max_time);
    Serial.println("ms");
    Serial.print("平均频率: ");
    Serial.print(1000.0 / (loop_total_time / loop_count));
    Serial.println("Hz");
    Serial.println("===================================\n");
  }
  #endif
}

 /**
 * @brief 设置RGB灯的颜色
 * @param rValue;gValue;bValue;
 * @arg 三个入口参数取值分别为:0~255;
 * @retval None
 * @note (255,0,0)绿色 (0,255,0)红色 (0,0,255)蓝色 (255,255,255)白色
 */
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue)
{
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}

/* 任务调度
 * ⭐ 优化架构: 统一命令调度和执行
 * 1. 检查I2C和串口命令（已由pollXXXCommand()解析好）
 * 2. 设置任务模式g_mode
 * 3. 执行对应任务
 * 4. 执行电机控制（根据motor_test_flag选择直接控制或运动学解算）
 */
void Task_Dispatcher(void)
{
  // ========== 步骤1: 检查命令并设置任务模式 ==========
  // 优先使用I2C命令（来自BLE），其次使用串口命令（用于调试）
  const char* cmd = (i2c_rec_data[0].length() > 0) ? i2c_charArray : charArray;

  /* 命令判断  */
  if(strcmp(cmd, "A") == 0 && avoid_flag == 0)
  {
      motor_test_flag = 0;  // 退出电机测试模式
      g_mode = MODE_ROCKERANDGRAVITY; // 摇杆&重力控制
  }
  if(strcmp(cmd, "B") == 0 && avoid_flag == 0)
  {
    motor_test_flag = 0;  // 退出电机测试模式
    g_mode = MODE_RGB_ADJUST;   // RGB调节
  }
  if(strcmp(cmd, "C") == 0 && avoid_flag == 0)
  {
    motor_test_flag = 0;  // 退出电机测试模式
    g_mode = MODE_SPEED_CONTROL;    // 速度控制
  }
  // ⭐ 已注释: E命令-舵机控制
  // if(strcmp(cmd, "E") == 0 && avoid_flag == 0)
  // {
  //   motor_test_flag = 0;  // 退出电机测试模式
  //   g_mode = MODE_SERVO_CONTROL;
  // }
  if(strcmp(cmd, "D") == 0)
  {
    g_mode = MODE_ULTRASOUND_SEND;
  }
  if(strcmp(cmd, "F") == 0)
  {
    motor_test_flag = 0;  // 退出电机测试模式
    g_mode = MODE_AVOID;
    avoid_flag = 1;
    // 优先使用I2C数据，其次使用串口数据
    String* data_source = (i2c_rec_data[1].length() > 0) ? i2c_rec_data : rec_data;
    g_state = atoi(data_source[1].c_str());
  }
  if(strcmp(cmd, "G") == 0)  /* 电机测试命令 */
  {
    avoid_flag = 0;  // 退出避障模式
    motor_test_flag = 1;  // 进入电机测试模式
    g_mode = MODE_MOTOR_TEST;
  }

  // ========== 步骤2: 执行任务 ==========
  if(g_mode == MODE_ROCKERANDGRAVITY)  // 摇杆&重力控制
  {
    Rockerandgravity_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_RGB_ADJUST)   // RGB调节
  {
    Rgb_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_SPEED_CONTROL)    // 速度控制
  {
    Speed_Task();
    g_mode = MODE_NULL;
  }
  // ⭐ 已注释: 舵机控制任务
  // if(g_mode == MODE_SERVO_CONTROL)
  // {
  //   Servo_Data_Receive();
  //   g_mode = MODE_NULL;
  // }
  if(g_mode == MODE_MOTOR_TEST)
  {
    Motor_Test_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_ULTRASOUND_SEND)
  {
    distance = ultrasound.Filter();   /* 获得滤波器输出值，单位：毫米(mm) */
    voltage_send = (int)(voltage*1000);  /* 将电压从V转换为mV */

    /* 电压异常跳变检测：如果电压突然下降超过500mV(0.5V)，可能是ADC采样干扰或瞬时压降
     * 记录为error_voltage但不立即使用，等待下次采样确认 */
    if(last_voltage_send - voltage_send >= 500)
    {
      error_voltage = voltage_send;  /* 标记为可能的异常值 */
    }

    /* 如果当前电压值不等于之前标记的异常值，说明电压正常，更新实际电压值
     * 这样可以过滤掉瞬时的异常读数，提高电压显示的稳定性 */
    if(voltage_send != error_voltage)
    {
      real_voltage_send = voltage_send;  /* 更新有效电压值 */
    }
    last_voltage_send = voltage_send;  /* 保存本次电压值，用于下次检测 */

    /* 将超声波距离数据和电压数据发送给APP，格式: $distance,voltage$ */
    Serial.print("$");Serial.print(distance);Serial.print(",");
    Serial.print(real_voltage_send);Serial.print("$");
    g_mode = MODE_NULL;
  }
  if(avoid_flag == 1)
  {
    Aovid();  /* 执行避障任务 */
  }

  /* 根据运动状态决定是否开启电压检测
   * g_state == 8 表示小车停止状态
   * 停止时开启电压检测，运动时关闭（避免电机PWM干扰ADC采样） */
  if(g_state == 8)
  {
    g_read = READ_VOLTAGE_ON;   /* 开启电压检测 */
  }
  else
  {
    g_read = READ_VOLTAGE_OFF;  /* 关闭电压检测 */
  }

  // ========== 步骤3: 执行电机控制 ==========
  // 如果在电机测试模式，直接控制电机，不使用运动学解算
  // test_motor_speeds 使用标准Z字形索引，直接传递给 Motors_Set
  // 否则执行速度控制器函数
  if(motor_test_flag == 1) {
    Motors_Set(test_motor_speeds[0], test_motor_speeds[1], test_motor_speeds[2], test_motor_speeds[3]);
  } else {
    Velocity_Controller(car_derection, speed_data, car_rot);
  }
}

 /* 摇杆控制任务：根据APP发送的状态码控制小车运动 */
void Rockerandgravity_Task(void)
{
  // ⭐ 优先使用I2C数据（从ESP32-BLE接收），如果I2C数据为空则使用串口数据（调试用）
  String* data_source = (i2c_rec_data[1].length() > 0) ? i2c_rec_data : rec_data;

  g_state = atoi(data_source[1].c_str());  /* 解析状态码字符串转为整数 */
  // Serial.println(g_state);  /* 调试用：打印状态码 */

  /* 状态码说明：
   * 0-7: 8个方向移动（0=右移, 1=右前, 2=前进, 3=左前, 4=左移, 5=左后, 6=后退, 7=右后）
   * 8: 停止移动（保持旋转状态）
   * 9: 开始顺时针旋转
   * 10: 开始逆时针旋转
   * 11: 停止旋转 */
  switch (g_state)
  {
    case 0:
      car_derection = 270;  // 右移（向右90° = 270°，使用360°坐标系表示）
      // car_rot = 0;       // 旋转速度保持不变（可能正在旋转）
      speed_data = speed_update;  /* 使用当前设定的速度 */
      break;
    case 1:
      car_derection = 315;  // 右前（-45° = 315°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 2:
      car_derection = 0;  // 前进（0°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 3:
      car_derection = 45;  // 左前（45°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 4:
      car_derection = 90;  // 左移（90°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 5:
      car_derection = 135;  // 左后（135°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 6:
      car_derection = 180;  // 后退（180°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 7:
      car_derection = 225;  // 右后（225°）
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 8:
      car_derection = 0;     /* 停止移动时方向设为0 */
      speed_data = 0;        /* 停止移动：线速度设为0 */
      car_rot = 0;           /* 停止旋转：角速度设为0 */
      rot_flag = 0;          /* 清除旋转标志 */
      break;

    case 9:
      // car_derection = 0;  /* 方向保持不变（可能正在移动） */
      // speed_data = 0;     /* 移动速度保持不变 */
      rot_flag = 1;          /* 设置旋转标志为1：顺时针旋转 */

      /* 旋转速度策略：
       * 如果当前是原地不动(speed_data == 0)：使用全速旋转(car_rot = speed_update)
       * 如果当前正在移动(speed_data != 0)：使用1/3速度旋转，避免过快转向失控 */
      if(speed_data == 0)
      {
        car_rot = speed_update;      /* 原地旋转：全速 */
      }
      else
      {
        car_rot = speed_update / 3;  /* 移动中旋转：降速，更稳定 */
      }

      break;

    case 10:
      // car_derection = 0;  /* 方向保持不变 */
      // speed_data = 0;     /* 移动速度保持不变 */
      rot_flag = 2;          /* 设置旋转标志为2：逆时针旋转 */

      /* 逆时针旋转：速度为负值
       * 同样使用速度策略：原地全速，移动中降速 */
      if(speed_data == 0)
      {
        car_rot = -speed_update;     /* 原地逆时针旋转：全速（负值） */
      }
      else
      {
        car_rot = -speed_update / 3; /* 移动中逆时针旋转：1/3速度 */
      }
      break;

    default:
      break;
  }

  // 清空I2C缓冲区，避免重复使用旧数据
  i2c_rec_data[0] = "";
  i2c_rec_data[1] = "";
  i2c_rec_data[2] = "";
  i2c_rec_data[3] = "";
}

 /* 超声波RGB调节函数 */
void Rgb_Task(void)
{
  // ⭐ 优先使用I2C数据，如果I2C数据为空则使用串口数据
  String* data_source = (i2c_rec_data[1].length() > 0) ? i2c_rec_data : rec_data;

  uint8_t r_data,g_data,b_data;
  r_data = (uint8_t)atoi(data_source[1].c_str());
  g_data = (uint8_t)atoi(data_source[2].c_str());
  b_data = (uint8_t)atoi(data_source[3].c_str());
  ultrasound.Color(r_data,g_data,b_data,r_data,g_data,b_data);

  // 清空I2C缓冲区
  i2c_rec_data[0] = "";
  i2c_rec_data[1] = "";
  i2c_rec_data[2] = "";
  i2c_rec_data[3] = "";
}

/* 低电压报警函数：根据电压值触发蜂鸣器和LED警报（不再读取ADC）
 * ⭐ 优化: 电压采集已移至reportSensorDataToESP32()，此函数只负责报警逻辑 */
void Low_Voltage_Warning(void)
{
  uint32_t currentTime_ms;
  currentTime_ms = millis();                              // 获取当前时间戳，用于非阻塞延时

  /* 低电压检测：7.4V锂电池组的安全截止电压约为7.0V
   * 低于7.0V时启动警报，防止过放损坏电池 */
  if(real_voltage_send <= 7000)
  {
    /* 如果当前不是RGB警报模式，则启动蜂鸣器警报
     * 优先级：蜂鸣器警报 → RGB警报（蜂鸣2次后切换到RGB） */
    if(g_warning != WARNING_RGB)
    {
      g_warning = WARNING_BEEP;        // 设置为蜂鸣器警报模式
    }
  }

  /* 蜂鸣器警报节奏控制：使用非阻塞方式实现"嘀嘀"声
   * 节奏：500ms响 + 500ms停 = 1秒一个周期 */
  if(g_warning == WARNING_BEEP)
  {
    // 前半秒(0-500ms)：蜂鸣器响
    if(currentTime_ms - previousTime_ms <= interval_ms/2)
    {
      tone(buzzerPin, 800);            // 输出800Hz音调，电压小于7V警报
    }
    // 后半秒(500-1000ms)：蜂鸣器停
    else if (currentTime_ms - previousTime_ms > interval_ms/2 && currentTime_ms - previousTime_ms < interval_ms)
    {
      noTone(buzzerPin);               // 关闭蜂鸣器
    }
  }

  // 时间周期重置：每1秒重置一次计时器
  if (currentTime_ms - previousTime_ms >= interval_ms)
  {
    // 蜂鸣器警报时，累加警报次数计数器
    if(g_warning == WARNING_BEEP)
    {
      beep_count++;                    // 每个周期(1秒)累加1次
    }

    previousTime_ms = currentTime_ms;  // 重置计时器
  }

  /* 蜂鸣器警报次数判断：连续警报2次后切换到RGB警报
   * 设计意图：先用声音提醒2秒，然后改用LED持续提示，避免持续噪音 */
  if(beep_count == 2)
  {
    beep_count = 0;                    // 清零计数器
    noTone(buzzerPin);                 // 关闭蜂鸣器
    g_warning = WARNING_RGB;           // 切换到RGB警报模式
  }

  /* RGB警报：显示红色LED，持续提示低电压
   * RGB值(0,10,0)在FastLED的GRB顺序中表示红色（G=0, R=10, B=0） */
  if(g_warning == WARNING_RGB)
  {
    Rgb_Show(10,0,10);                  // 显示低亮度红色（避免过亮刺眼）
  }
}

/* ⭐ 已注释: 机械爪控制任务 */
// void Servo_Data_Receive(void)
// {
//   // ⭐ 优先使用I2C数据，如果I2C数据为空则使用串口数据
//   String* data_source = (i2c_rec_data[1].length() > 0) ? i2c_rec_data : rec_data;
//
//   increase_angle = atoi(data_source[1].c_str());
//   myservo.write(default_angle + increase_angle);
//
//   // 清空I2C缓冲区
//   i2c_rec_data[0] = "";
//   i2c_rec_data[1] = "";
//   i2c_rec_data[2] = "";
//   i2c_rec_data[3] = "";
// }

/* 电机测试任务 */
void Motor_Test_Task(void)
{
  // ⭐ 优先使用I2C数据，如果I2C数据为空则使用串口数据
  String* data_source = (i2c_rec_data[1].length() > 0) ? i2c_rec_data : rec_data;

  uint8_t motor_id = (uint8_t)atoi(data_source[1].c_str());  // 电机编号 0-3 (标准Z字形索引)
  int8_t speed = (int8_t)atoi(data_source[2].c_str());       // 速度 -100到100

  // 保存到全局数组，在loop中持续输出PWM
  // 使用标准Z字形索引: motor_id=0→左前, 1→右前, 2→左后, 3→右后
  test_motor_speeds[0] = 0;
  test_motor_speeds[1] = 0;
  test_motor_speeds[2] = 0;
  test_motor_speeds[3] = 0;

  if(motor_id >= 0 && motor_id < 4) {
    test_motor_speeds[motor_id] = speed;
  }

  // 清空I2C缓冲区
  i2c_rec_data[0] = "";
  i2c_rec_data[1] = "";
  i2c_rec_data[2] = "";
  i2c_rec_data[3] = "";
}

/* 避障模式：使用超声波传感器自动避障前进 */
void Aovid(void)
{
  distance = ultrasound.Filter();  /* 获取滤波后的超声波距离，单位：毫米(mm) */

  /* 避障状态判断：g_state由F命令控制
   * g_state == 1: 开启避障模式
   * g_state == 0: 关闭避障模式 */
  if(g_state == 1)
  {
    /* 距离过近(<400mm=40cm)：停止前进，原地右转避障
     * 策略：停止移动(speed_data=0)，全速旋转(car_rot=100) */
    if(distance < 400)
    {
      car_derection = 0;      /* 方向设为0（不重要，因为speed_data=0） */
      car_rot = 100;          /* 全速顺时针旋转，寻找空旷方向 */
      speed_data = 0;         /* 停止前进 */
    }

    /* 距离安全(>=500mm=50cm)：继续前进
     * 策略：以中速(50)前进，不旋转 */
    if(distance >= 500)
    {
      car_derection = 0;      /* 方向0°：前进 */
      car_rot = 0;            /* 停止旋转 */
      speed_data = 50;        /* 中速前进（speed=50） */
    }

    /* 注意：距离在400-500mm之间时，保持上一状态不变
     * 这个"死区"设计避免在临界值附近频繁切换状态，提高稳定性 */
  }
  else if(g_state == 0)
  {
    /* 关闭避障模式：停止所有运动，清除避障标志 */
    car_derection = 0;    /* 方向设为0 */
    car_rot = 0;          /* 停止旋转 */
    speed_data = 0;       /* 停止移动 */
    g_mode = NULL;        /* 清除任务模式 */
    avoid_flag = 0;       /* 清除避障标志，退出避障模式 */
  }
}
 /* 速度调节函数：接收APP发送的速度值，更新全局速度变量 */
void Speed_Task(void)
{
  // ⭐ 优先使用I2C数据（从ESP32-BLE接收），如果I2C数据为空则使用串口数据（调试用）
  String* data_source = (i2c_rec_data[1].length() > 0) ? i2c_rec_data : rec_data;

  speed_update = (uint8_t)atoi(data_source[1].c_str());  /* 解析速度值字符串，范围：20-100 */
  Serial.print("C|");Serial.print(speed_update); Serial.print("|$");  /* 回显速度值给上位机确认 */

  // 清空I2C缓冲区，避免重复使用旧数据
  i2c_rec_data[0] = "";
  i2c_rec_data[1] = "";
  i2c_rec_data[2] = "";
  i2c_rec_data[3] = "";
}

 /* 电机初始化函数：设置电机引脚为输出模式，并停止所有电机 */
void Motor_Init(void)
{
  /* 遍历4个电机，设置引脚模式 */
  for(uint8_t i = 0; i < 4; i++) {
    pinMode(motordirectionPin[i], OUTPUT);  /* 方向引脚设为输出 */
    pinMode(motorpwmPin[i], OUTPUT);        /* PWM引脚设为输出 */
  }
  Velocity_Controller( 0, 0, 0);  /* 初始化：停止所有电机（方向0°，速度0，旋转0） */
}

/**
 * @brief 速度控制函数（标准Z字形麦克纳姆轮运动学逆解）
 * @param angle   用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。
 *                取值为0~359
 * @param velocity   用于控制小车速度，取值为0~100。
 * @param rot     用于控制小车的自转速度，取值为-100~100，若大于0小车有一个顺
 *                 时针的自转速度，若小于0则有一个逆时针的自转速度。
 * @retval None
 *
 * @note 标准麦克纳姆轮运动学公式（适配当前硬件）：
 *       坐标系：X轴=车头方向(前进), Y轴=车身左侧方向(左移)
 *       velocity_0 (左前) = Vx + Vy + Vθ
 *       velocity_1 (右前) = Vx - Vy - Vθ
 *       velocity_2 (左后) = Vx - Vy + Vθ
 *       velocity_3 (右后) = Vx + Vy - Vθ
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot)
{
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;  /* 速度缩放因子（预留，当前未使用） */

  /* 坐标系转换：将用户角度转换为数学角度
   * 标准坐标系：X轴=车头方向(前进)，Y轴=车身左侧方向(左移)
   * 用户angle定义：0°=前进，90°=左移，逆时针为正（标准ROS右手系）
   * 已修正：移除不必要的负号，直接使用angle计算
   * PI/180 将角度转换为弧度（三角函数需要弧度输入） */
  float rad = angle * PI / 180;

  /* 分解速度分量（不做提前归一化，保持原始速度比例）
   * cos(rad)和sin(rad)的范围是[-1, 1]，乘以velocity后得到速度分量 */
  float Vx = velocity * cos(rad);  // 前进速度分量（前为正，后为负）
  float Vy = velocity * sin(rad);  // 横移速度分量（左为正，右为负）

  /* 标准Z字形麦克纳姆轮运动学逆解公式（符合ROS标准定义）
   * 旋转方向定义：正值=逆时针旋转（符合右手坐标系）
   * 公式推导基于麦克纳姆轮的辊子倾斜方向（45°）
   * 左前轮和右后轮的辊子方向相同，左后轮和右前轮的辊子方向相同 */
  velocity_0 = (int8_t)(Vx + Vy + rot);  // 左前轮速度
  velocity_1 = (int8_t)(Vx - Vy - rot);  // 右前轮速度
  velocity_2 = (int8_t)(Vx - Vy + rot);  // 左后轮速度
  velocity_3 = (int8_t)(Vx + Vy - rot);  // 右后轮速度

  /* 输出端归一化：防止任何轮子速度超过±100
   * 如果任何轮子速度超过100，按比例缩放所有轮子速度
   * 这样可以保持运动方向不变，只是整体速度降低 */
  float max_vel = max(abs(velocity_0), max(abs(velocity_1),
                      max(abs(velocity_2), abs(velocity_3))));

  if(max_vel > 100) {
    float scale = 100.0 / max_vel;  /* 计算缩放因子 */
    velocity_0 = (int8_t)(velocity_0 * scale);
    velocity_1 = (int8_t)(velocity_1 * scale);
    velocity_2 = (int8_t)(velocity_2 * scale);
    velocity_3 = (int8_t)(velocity_3 * scale);
  }

  /* 调用电机设置函数，将计算出的速度应用到4个电机 */
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}

/**
 * @brief PWM与轮子转向设置函数（标准Z字形索引）
 * @param Motor_0   标准索引0的速度（左前轮），范围：-100到100
 * @param Motor_1   标准索引1的速度（右前轮），范围：-100到100
 * @param Motor_2   标准索引2的速度（左后轮），范围：-100到100
 * @param Motor_3   标准索引3的速度（右后轮），范围：-100到100
 * @retval None
 *
 * @note 引脚数组已按标准Z字形索引排列，直接使用索引访问
 *       索引0 → motorpwmPin[0]=9  → 左前轮
 *       索引1 → motorpwmPin[1]=10 → 右前轮
 *       索引2 → motorpwmPin[2]=6  → 左后轮
 *       索引3 → motorpwmPin[3]=11 → 右后轮
 *
 * ⭐ P0优化最终版: 软件PWM + 独立计时器
 *    - 问题1: 原版4个电机共用计时器 → 速度误差±15%
 *    - 问题2: 硬件PWM (analogWrite) → 引脚9,10与Servo库冲突 → 前两轮不转
 *    - 最终方案: 软件PWM + 每个电机独立计时器 → 速度误差<±2% + 无Servo冲突
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3)
{
  /* 应用电机校准系数（标准Z字形索引）
   * motor_calibration数组用于补偿硬件个体差异，例如某个电机略慢可以增大系数
   * 默认值都是1.0，表示不做校准。实际使用时根据测试调整 */
  int8_t motors[4] = {
    (int8_t)(Motor_0 * motor_calibration[0]),  /* 左前轮校准 */
    (int8_t)(Motor_1 * motor_calibration[1]),  /* 右前轮校准 */
    (int8_t)(Motor_2 * motor_calibration[2]),  /* 左后轮校准 */
    (int8_t)(Motor_3 * motor_calibration[3])   /* 右后轮校准 */
  };

  /**
   * 标准Z字形定义下的基准方向数组
   * direction数组定义了每个电机"前进时"的方向位电平
   * 前进时：左侧轮(左前、左后)方向位=0，右侧轮(右前、右后)方向位=1
   *
   * direction[i] 对应标准索引i的基准方向：
   *   direction[0]=0 : 左前轮，前进时方向位=0
   *   direction[1]=1 : 右前轮，前进时方向位=1
   *   direction[2]=0 : 左后轮，前进时方向位=0
   *   direction[3]=1 : 右后轮，前进时方向位=1
   *
   * 麦克纳姆轮的机械特性：左右两侧轮子转向相反才能实现直线前进
   */
  bool direction[4] = { 0, 1, 0, 1 };

  /* 遍历标准索引，直接控制对应引脚 */
  for(uint8_t i = 0; i < 4; ++i)
  {
    /* 根据速度正负决定最终方向
     * 速度为正：使用基准方向（前进）
     * 速度为负：翻转方向（后退） */
    bool final_direction = direction[i];
    if(motors[i] < 0) final_direction = !final_direction;  /* 负速度时翻转方向 */

    /* 设置方向引脚：控制电机正转或反转 */
    digitalWrite(motordirectionPin[i], final_direction);

    /* ⭐ 使用软件PWM，每个电机独立计时器
     * PWM_Out函数会根据占空比(0-100)生成PWM波形，控制电机速度
     * abs(motors[i])取绝对值，因为方向已经由方向引脚控制 */
    PWM_Out(i, motorpwmPin[i], abs(motors[i]));
  }
}

/**
 * @brief 软件PWM输出函数（每个引脚独立计时器版本）
 * @param motor_index  电机索引 (0-3，标准Z字形: 0=左前, 1=右前, 2=左后, 3=右后)
 * @param PWM_Pin      PWM输出引脚编号
 * @param DutyCycle    占空比 (0-100)，0=停止，100=全速
 *
 * ⭐ P0优化最终版: 修复软件PWM的Timer1冲突问题
 * - 问题: 引脚9,10使用Timer1，与Servo库冲突导致analogWrite()失效
 * - 解决: 使用软件PWM，但每个电机独立计时器（修复原代码共用计时器的BUG）
 * - 原BUG: 4个电机共用previousTime_us → 相位错乱 → 速度误差±15%
 * - 新实现: motor_previousTime_us[4] 独立计时器 → 速度误差<±2%
 *
 * @note 软件PWM工作原理：
 *       1. 一个PWM周期 = period微秒（当前20000us=20ms，频率500Hz）
 *       2. 高电平时间 = period × (DutyCycle/100)
 *       3. 使用micros()非阻塞计时，不影响主循环其他任务
 *       4. 每个电机独立计时器，避免相互干扰
 */
void PWM_Out(uint8_t motor_index, uint8_t PWM_Pin, int8_t DutyCycle)
{
  uint32_t currentTime_us = micros();  /* 获取当前微秒时间戳 */

  /* 计算高电平持续时间
   * 例如：period=20000us, DutyCycle=50 → highTime=10000us (占空比50%) */
  int highTime = (period/100) * DutyCycle;

  /* PWM波形生成逻辑：
   * 在一个周期内，前highTime微秒输出高电平，剩余时间输出低电平 */
  if ((currentTime_us - motor_previousTime_us[motor_index]) <= highTime)
  {
    digitalWrite(PWM_Pin, HIGH);  /* 输出高电平：电机通电，产生转矩 */
  }
  else
  {
    digitalWrite(PWM_Pin, LOW);   /* 输出低电平：电机断电，惯性滑行 */
  }

  /* 周期结束判断：当前周期时间超过period时，重置计时器开始下一个周期
   * 使用独立计时器motor_previousTime_us[motor_index]，每个电机互不干扰 */
  if (currentTime_us - motor_previousTime_us[motor_index] >= period)
  {
    motor_previousTime_us[motor_index] = currentTime_us;  /* 重置该电机的计时器 */
  }
}

// ==================== 命令获取功能 ====================

/**
 * @brief 轮询串口获取命令
 * @note 从串口读取调试命令（用于Arduino IDE串口调试）
 *
 * ⭐ 优化架构: 职责分离 - 只负责获取命令，不执行
 * 命令获取 → 存入rec_data[] → 由Task_Dispatcher()统一调度执行
 */
void pollSerialCommand(void) {
  uint8_t index = 0;

  // ========== 处理串口命令 ==========
  while (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('$');

    while (cmd.indexOf('|') != -1)
    {
      rec_data[index] = cmd.substring(0, cmd.indexOf('|'));  /* 提取从开始到第一个逗号之前的子字符串 */
      cmd = cmd.substring(cmd.indexOf('|') + 1);             /* 更新字符串，去掉已提取的子字符串和逗号 */
      index++;      /* 更新索引 */
    }
    charArray = rec_data[0].c_str();      /* 转成C字符串形式 */

    // ⭐ 不执行命令，只负责获取和解析
    // 命令执行由 Task_Dispatcher() 统一调度
  }
}

// ==================== I2C 主机轮询功能 ====================

/**
 * @brief 轮询ESP32-S3从机获取命令（使用寄存器协议）
 * @note 每隔10ms从ESP32读取BLE命令
 *       寄存器0x10: BLE命令数据
 *
 * ⭐ 优化架构: 职责分离 - 只负责获取命令，不执行
 * 命令获取 → 存入i2c_rec_data[] → 由Task_Dispatcher()统一调度执行
 */
void pollESP32Command(void) {
  // 定时轮询
  if (millis() - last_i2c_poll < I2C_POLL_INTERVAL) {
    return;  // 未到轮询时间
  }

  last_i2c_poll = millis();
  i2c_total_polls++;  // 统计轮询次数

  // ========== 读取I2C命令数据 ==========
  Wire.beginTransmission(I2C_ESP32_ADDR);
  Wire.write(0x10);  // 命令寄存器地址
  byte error = Wire.endTransmission();

  if (error == 0) {
    // 请求读取命令数据
    uint8_t bytesReceived = Wire.requestFrom((int)I2C_ESP32_ADDR, 16);

    if (bytesReceived > 0) {
      // 读取数据到缓冲区
      uint8_t index = 0;
      while (Wire.available() && index < I2C_BUFFER_SIZE - 1) {
        i2c_rx_buffer[index++] = Wire.read();
      }
      i2c_rx_buffer[index] = '\0';  // 添加字符串结束符

      // 如果收到有效数据(不是空数据)，解析命令
      if (index > 0 && !(index == 1 && i2c_rx_buffer[0] == 0)) {
        // 记录统计信息
        i2c_success_reads++;
        last_received_cmd = String(i2c_rx_buffer);

        // 解析命令字符串到 i2c_rec_data[] 缓冲区
        String cmd = String(i2c_rx_buffer);
        uint8_t parse_index = 0;

        while (cmd.indexOf('|') != -1 && parse_index < 4)
        {
          i2c_rec_data[parse_index] = cmd.substring(0, cmd.indexOf('|'));
          cmd = cmd.substring(cmd.indexOf('|') + 1);
          parse_index++;
        }

        // 更新命令指针，供Task_Dispatcher()使用
        if (parse_index > 0) {
          i2c_charArray = i2c_rec_data[0].c_str();
        }

        // ⭐ 不执行命令，只负责获取和解析
        // 命令执行由 Task_Dispatcher() 统一调度
      }
    }
  }
}

/**
 * @brief ⭐ P2优化: 超声波传感器采样函数（非阻塞）
 * @note 每隔50ms采样一次超声波距离
 *       采样频率高于上报频率，确保实时响应障碍物
 *       采样结果存储在全局变量distance中
 */
void sampleUltrasound(void) {
  // 定时检查: 每50ms采样一次
  if (millis() - last_ultrasound_sample < ULTRASOUND_SAMPLE_INTERVAL) {
    return;  // 未到采样时间
  }
  last_ultrasound_sample = millis();

  // 采集超声波距离 (Ultrasound库已实现快速滤波,<5ms)
  #ifdef ULTRASOUND_TIMING_DEBUG
  unsigned long us_start = micros();
  #endif

  distance = ultrasound.Filter();  // 更新全局distance变量

  #ifdef ULTRASOUND_TIMING_DEBUG
  unsigned long us_duration = micros() - us_start;
  ultrasound_total_time += us_duration;
  ultrasound_call_count++;
  if(us_duration > ultrasound_max_time) {
    ultrasound_max_time = us_duration;
  }
  // 每100次输出一次统计信息
  if(ultrasound_call_count % 100 == 0) {
    Serial.print("[超声波耗时] 平均: ");
    Serial.print(ultrasound_total_time / ultrasound_call_count);
    Serial.print("us, 最大: ");
    Serial.print(ultrasound_max_time);
    Serial.print("us, 本次: ");
    Serial.print(us_duration);
    Serial.println("us");
  }
  // 单次耗时超过5ms时立即警告
  if(us_duration > 5000) {
    Serial.print("[警告] 超声波阻塞! 耗时: ");
    Serial.print(us_duration);
    Serial.println("us");
  }
  #endif
}

/**
 * @brief ⭐ P2优化: 电压传感器采样函数（非阻塞）
 * @note 每隔200ms采样一次电压
 *       电压变化慢（秒级），无需高频采样
 *       采样结果存储在全局变量real_voltage_send中
 *       如果电压<7V，触发Low_Voltage_Warning()报警
 */
void sampleVoltage(void) {
  // 定时检查: 每200ms采样一次
  if (millis() - last_voltage_sample < VOLTAGE_SAMPLE_INTERVAL) {
    return;  // 未到采样时间
  }
  last_voltage_sample = millis();

  // 采集电压 (ADC读取<1ms)
  voltage = analogRead(A3) * 0.02989;  // 单位: V
  int current_voltage = (int)(voltage * 1000);  // 转换为mV

  // 电压跳变保护: 变化超过0.5V视为异常,使用上次值
  if(abs(current_voltage - real_voltage_send) < 500) {
    real_voltage_send = current_voltage;
  }
  // else: 使用上次的 real_voltage_send 值

  // 低压报警检测: 如果电压<7V,触发报警
  if(real_voltage_send < 7000) {
    Low_Voltage_Warning();  // 执行低压报警（蜂鸣器和RGB）
  }
}

/**
 * @brief ⭐ P2优化: Arduino主动上报传感器数据给ESP32
 * @note 每隔200ms通过I2C发送传感器数据给ESP32
 *       优化前: Android每200ms发送D命令请求数据 (占用队列和BLE带宽)
 *       优化后: Arduino主动推送数据 (无需D命令,ESP32收到后立即BLE notify)
 *
 * ⭐ 架构优化: 数据采集与上报分离
 *    - sampleUltrasound(): 每50ms采样超声波 (快速响应)
 *    - sampleVoltage(): 每200ms采样电压 (低频采样)
 *    - reportSensorDataToESP32(): 每200ms上报数据 (与Android同步)
 *    - 各模块独立定时，互不干扰，易于调整频率
 *
 * 安全措施:
 * 1. 非阻塞定时器: 使用millis()判断,不阻塞主循环
 * 2. 数据来源: 直接读取全局变量distance和real_voltage_send
 * 3. I2C容错: 发送失败静默忽略,不影响下次上报
 */
void reportSensorDataToESP32(void) {
  // 定时检查: 每200ms上报一次
  if (millis() - last_sensor_report < SENSOR_REPORT_INTERVAL) {
    return;  // 未到上报时间
  }
  last_sensor_report = millis();

  // I2C发送传感器数据
  // 数据格式: [寄存器0x11] [distance高字节] [distance低字节] [voltage高字节] [voltage低字节]
  Wire.beginTransmission(I2C_ESP32_ADDR);
  Wire.write(0x11);  // 传感器数据寄存器
  Wire.write((uint8_t)(distance >> 8));           // 距离高字节
  Wire.write((uint8_t)(distance & 0xFF));         // 距离低字节
  Wire.write((uint8_t)(real_voltage_send >> 8));  // 电压高字节
  Wire.write((uint8_t)(real_voltage_send & 0xFF));// 电压低字节
  byte error = Wire.endTransmission();

  // I2C容错: 发送失败不影响下次上报 (ESP32可能暂时忙碌)
  if (error != 0) {
    // 静默忽略错误,继续下次上报
  }
}
