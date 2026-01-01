/**
 * @file app_control_common.ino
 * @author Anonymity(Anonymity@hiwonder.com)
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
 *   用户angle定义：0°=前进，90°=右移，逆时针为正
 *
 * 运动学公式：
 *   V_左前(0) = Vx + Vy - Vθ
 *   V_右前(1) = Vx - Vy + Vθ
 *   V_左后(2) = Vx - Vy - Vθ
 *   V_右后(3) = Vx + Vy + Vθ
 *
 * 其中：
 *   Vx = velocity * cos(-angle) : 前进速度分量
 *   Vy = velocity * sin(-angle) : 横移速度分量（左为正）
 *   Vθ = rot                    : 旋转速度（逆时针为正）
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
#include <Servo.h>
#include "Ultrasound.h"

typedef enum {
  MODE_NULL,
  MODE_ROCKERANDGRAVITY,  /* 摇杆&重力控制 */
  MODE_RGB_ADJUST,        /* RGB调节 */
  MODE_SPEED_CONTROL,     /* 速度控制 */
  MODE_ULTRASOUND_SEND,   /* 发送超声波距离给上位机 */
  MODE_SERVO_CONTROL,     /* 机械爪控制 */
  MODE_VOLTAGE_SEND,      /* 发送电压值给APP */
  MODE_AVOID,             /* 避障 */
  MODE_MOTOR_TEST         /* 电机测试 */
} CarMode;

typedef enum {
  WARNING_OFF,
  WARNING_BEEP,
  WARNING_RGB,
} VoltageWarning;

typedef enum {
  READ_VOLTAGE_ON,
  READ_VOLTAGE_OFF
} ReadVoltageState;

Servo myservo;          /* 实例化舵机 */
Ultrasound ultrasound;  /* 实例化超声波 */

static VoltageWarning g_warning = WARNING_OFF;
static CarMode g_mode = MODE_NULL;
static ReadVoltageState g_read = READ_VOLTAGE_ON;

static uint8_t g_state = 8;         /* 接收的APP子指令 */
static uint8_t avoid_flag = 0;      /* 避障模式开关标志位 */
static uint8_t motor_test_flag = 0; /* 电机测试模式标志位 */
static uint8_t rot_flag = 0;         /* 转向标志位 */
static uint8_t beep_count = 0;      /* 蜂鸣器鸣响次数 */

static int car_derection = 0;       /* 设置小车移动的角度 */
static int8_t car_rot = 0;          /* 设置小车角速度 */
static uint8_t speed_data = 0;      /* 设置小车线速度 */
static uint8_t speed_update = 50;   /* APP更新的线速度 */

/* 电压监测相关参数 */
static float voltage;
static int voltage_send;
static int last_voltage_send;
static int real_voltage_send;
static int error_voltage;

/* 电机测试相关参数 */
static int8_t test_motor_speeds[4] = {0, 0, 0, 0}; /* 测试模式下的电机速度（标准Z字形索引） */

static CRGB rgbs[1];
String rec_data[4];                 /* 接收APP的发送数据 */

const char *charArray;

/* 引脚定义 - 按标准Z字形索引顺序 */
const static uint8_t ledPin = 2;
const static uint8_t buzzerPin = 3;
const static uint8_t servoPin = 5;
const static uint8_t motorpwmPin[4] = { 9, 10, 6, 11} ;        /* 标准Z字形索引：0=左前, 1=右前, 2=左后, 3=右后 */
const static uint8_t motordirectionPin[4] = { 8, 12, 7, 13};   /* 方向引脚对应PWM引脚顺序 */

const static int pwmFrequency = 500;                /* PWM频率，单位是赫兹 */
const static int period = 10000000 / pwmFrequency;  /* PWM周期，单位是微秒 */
const static uint32_t interval_us = 20000;          /* 微秒计数时间间隔 用于非阻塞延时 */
const static uint32_t interval_ms = 1000;           /* 毫秒计数时间间隔 用于非阻塞延时 */

static uint32_t previousTime_us = 0;          /* 上一次的微秒计数时间间隔 用于非阻塞延时 */
static uint32_t previousTime_ms = 0;          /* 上一次的毫秒计数时间间隔 用于非阻塞延时 */

static int increase_angle = 0;                   /* 设置舵机角度 */
static int default_angle = 90;
static uint16_t distance = 0;                 /* 超声波距离 */

void Aovid(void);
void Rgb_Task(void);
void Motor_Init(void);
void Speed_Task(void);
void Task_Dispatcher(void);
void Servo_Data_Receive(void);
void Motor_Test_Task(void);
void Rockerandgravity_Task(void);
void Voltage_Detection_Task(void);
void PWM_Out(uint8_t PWM_Pin, int8_t DutyCycle);
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, ledPin, RGB>(rgbs, 1);
  Motor_Init();
  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);                   /* 绑定舵机指定引脚 */
  myservo.write(default_angle + increase_angle);                 /* 写入舵机角度 */
  tone(buzzerPin, 1200);                      /* 输出音调信号的函数,频率为1200 */
  delay(100);
  noTone(buzzerPin);
  voltage_send = analogRead(A3)*0.02989*1000;   /* 电压计算 */
  last_voltage_send = voltage_send;
  real_voltage_send = voltage_send;
}

void loop() {
  // 如果在电机测试模式，直接控制电机，不使用运动学解算
  // test_motor_speeds 使用标准Z字形索引，直接传递给 Motors_Set
  if(motor_test_flag == 1) {
    Motors_Set(test_motor_speeds[0], test_motor_speeds[1], test_motor_speeds[2], test_motor_speeds[3]);
  } else {
    Velocity_Controller(car_derection, speed_data, car_rot);
  }

  Task_Dispatcher();

  if(g_read == READ_VOLTAGE_ON)
  {
    Voltage_Detection();
  }
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

/* 任务调度 */
void Task_Dispatcher(void)
{
  uint8_t index = 0;
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
    if(strcmp(charArray, "A") == 0 && avoid_flag == 0)  /* 命令判断  */
    {
        motor_test_flag = 0;  // 退出电机测试模式
        g_mode = MODE_ROCKERANDGRAVITY;
    }
    if(strcmp(charArray, "B") == 0 && avoid_flag == 0)
    {
      motor_test_flag = 0;  // 退出电机测试模式
      g_mode = MODE_RGB_ADJUST;
    }
    if(strcmp(charArray, "C") == 0 && avoid_flag == 0)
    {
      motor_test_flag = 0;  // 退出电机测试模式
      g_mode = MODE_SPEED_CONTROL;
    }
    if(strcmp(charArray, "E") == 0 && avoid_flag == 0)
    {
      motor_test_flag = 0;  // 退出电机测试模式
      g_mode = MODE_SERVO_CONTROL;
    }
    if(strcmp(charArray, "D") == 0)
    {
      g_mode = MODE_ULTRASOUND_SEND;
    }
    if(strcmp(charArray, "F") == 0)
    {
      motor_test_flag = 0;  // 退出电机测试模式
      g_mode = MODE_AVOID;
      avoid_flag = 1;
      g_state = atoi(rec_data[1].c_str());
    }
    if(strcmp(charArray, "G") == 0)  /* 电机测试命令 */
    {
      avoid_flag = 0;  // 退出避障模式
      motor_test_flag = 1;  // 进入电机测试模式
      g_mode = MODE_MOTOR_TEST;
    }
  }
  if(g_mode == MODE_ROCKERANDGRAVITY)
  {
    Rockerandgravity_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_RGB_ADJUST)
  {
    Rgb_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_SPEED_CONTROL)
  {
    Speed_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_SERVO_CONTROL)
  {
    Servo_Data_Receive();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_MOTOR_TEST)
  {
    Motor_Test_Task();
    g_mode = MODE_NULL;
  }
  if(g_mode == MODE_ULTRASOUND_SEND)
  {
    distance = ultrasound.Filter();   /* 获得滤波器输出值 */
    voltage_send = (int)(voltage*1000);
    if(last_voltage_send - voltage_send >= 500)
    {
      error_voltage = voltage_send;
    }
    if(voltage_send != error_voltage)
    {
      real_voltage_send = voltage_send;
    }
    last_voltage_send = voltage_send;

    /* 将超声波距离数据和电压数据发送给APP */
    Serial.print("$");Serial.print(distance);Serial.print(",");
    Serial.print(real_voltage_send);Serial.print("$");
    g_mode = MODE_NULL;
  }
  if(avoid_flag == 1)
  {
    Aovid();
  }
  if(g_state == 8)
  {
    g_read = READ_VOLTAGE_ON;
  }
  else
  {
    g_read = READ_VOLTAGE_OFF;
  }
}

 /* 摇杆控制任务 */
void Rockerandgravity_Task(void)
{
  g_state = atoi(rec_data[1].c_str());
  // Serial.println(g_state);
  switch (g_state)
  {
    case 0:
      car_derection = 90;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 1:
      car_derection = 45;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 2:
      car_derection = 0;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 3:
      car_derection = 315;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 4:
      car_derection = 270;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 5:
      car_derection = 225;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 6:
      car_derection = 180;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 7:
      car_derection = 135;
      // car_rot = 0;
      speed_data = speed_update;
      break;
    case 8:
      car_derection = 0;
      // car_rot = 0;
      speed_data = 0;
      car_rot = rot_flag == 1 ? speed_update: (rot_flag == 2 ? -speed_update : 0);
      break;
    case 9:
      // car_derection = 0;
      // speed_data = 0;
      rot_flag = 1;
      if(speed_data == 0)
      {
        car_rot = speed_update;
      }
      else
      {
        car_rot = speed_update / 3;
      }

      break;

    case 10:
      // car_derection = 0;
      // speed_data = 0;
      rot_flag = 2;
      if(speed_data == 0)
      {
        car_rot = -speed_update;
      }
      else
      {
        car_rot = -speed_update / 3;
      }
      break;

    case 11:
      // car_derection = 0;
      car_rot = 0;
      rot_flag = 0;
      break;

    default:
      break;
  }
}

 /* 超声波RGB调节函数 */
void Rgb_Task(void)
{
  uint8_t r_data,g_data,b_data;
  r_data = (uint8_t)atoi(rec_data[1].c_str());
  g_data = (uint8_t)atoi(rec_data[2].c_str());
  b_data = (uint8_t)atoi(rec_data[3].c_str());
  ultrasound.Color(r_data,g_data,b_data,r_data,g_data,b_data);
}

/* 电压监测 */
void Voltage_Detection(void)
{
  uint32_t currentTime_ms;
  currentTime_ms = millis();
  voltage = analogRead(A3)*0.02989;   /* 电压计算 */

  // 修复：每次检测时都更新 real_voltage_send，避免使用 setup() 时的异常初始值
  int current_voltage = (int)(voltage * 1000);
  if(abs(current_voltage - real_voltage_send) < 2000) {  // 电压变化不超过2V才更新（防止异常跳变）
    real_voltage_send = current_voltage;
  }

  if(real_voltage_send <= 7000)
  {
    if(g_warning != WARNING_RGB)
    {
      g_warning = WARNING_BEEP;
    }
  }
  if(g_warning == WARNING_BEEP)
  {
    if(currentTime_ms - previousTime_ms <= interval_ms/2)
    {
      tone(buzzerPin, 800);  /* 电压小于7V蜂鸣器警报 */
    }
    else if (currentTime_ms - previousTime_ms > interval_ms/2 && currentTime_ms - previousTime_ms < interval_ms)
    {
      noTone(buzzerPin);
    }
  }
  if (currentTime_ms - previousTime_ms >= interval_ms)
  {
    if(g_warning == WARNING_BEEP)
    {
      beep_count++;
    }

    previousTime_ms = currentTime_ms;
  }
  if(beep_count == 2)
  {
    beep_count = 0;
    noTone(buzzerPin);
    g_warning = WARNING_RGB;
  }
  if(g_warning == WARNING_RGB)
  {
    Rgb_Show(0,10,0);
  }
}

/* 机械爪控制任务 */
void Servo_Data_Receive(void)
{
  increase_angle = atoi(rec_data[1].c_str());
  myservo.write(default_angle + increase_angle);
}

/* 电机测试任务 */
void Motor_Test_Task(void)
{
  uint8_t motor_id = (uint8_t)atoi(rec_data[1].c_str());  // 电机编号 0-3 (标准Z字形索引)
  int8_t speed = (int8_t)atoi(rec_data[2].c_str());       // 速度 -100到100

  // 保存到全局数组，在loop中持续输出PWM
  // 使用标准Z字形索引: motor_id=0→左前, 1→右前, 2→左后, 3→右后
  test_motor_speeds[0] = 0;
  test_motor_speeds[1] = 0;
  test_motor_speeds[2] = 0;
  test_motor_speeds[3] = 0;

  if(motor_id >= 0 && motor_id < 4) {
    test_motor_speeds[motor_id] = speed;
  }
}

/* 避障模式 */
void Aovid(void)
{
  distance = ultrasound.Filter();
  if(g_state == 1)
  {
    if(distance < 400)
    {
      car_derection = 0;
      car_rot = 100;
      speed_data = 0;
    }
    if(distance >= 500)
    {
      car_derection = 0;
      car_rot = 0;
      speed_data = 50;
    }
  }
  else if(g_state == 0)
  {
    car_derection = 0;
    car_rot = 0;
    speed_data = 0;
    g_mode = NULL;
    avoid_flag = 0;
  }
}
 /* 速度调节调节函数 */
void Speed_Task(void)
{
  speed_update = (uint8_t)atoi(rec_data[1].c_str());
  Serial.print("C|");Serial.print(speed_update); Serial.print("|$");
}

 /* 电机初始化函数 */
void Motor_Init(void)
{
  for(uint8_t i = 0; i < 4; i++) {
    pinMode(motordirectionPin[i], OUTPUT);
    pinMode(motorpwmPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0);
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
 *       velocity_0 (左前) = Vx + Vy - Vθ
 *       velocity_1 (右前) = Vx - Vy + Vθ
 *       velocity_2 (左后) = Vx - Vy - Vθ
 *       velocity_3 (右后) = Vx + Vy + Vθ
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot)
{
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;

  // 标准坐标系：X轴=车头方向(前进)，Y轴=车身左侧方向(左移)
  // 用户angle定义：0°=前进，90°=右移，逆时针为正
  // 数学坐标系转换：取负号将用户的"逆时针为正"转为标准的"右手坐标系"
  float rad = -angle * PI / 180;

  // 速度因子调整（旋转时降速50%）
  if (rot == 0) speed = 1;
  else speed = 0.5;

  // 速度归一化（考虑麦轮45°滚轮角度）
  velocity /= sqrt(2);

  // 分解速度分量
  float Vx = velocity * cos(rad);  // 前进速度分量（前为正）
  float Vy = velocity * sin(rad);  // 横移速度分量（左为正）

  // 标准Z字形麦克纳姆轮运动学逆解公式（适配当前硬件）
  velocity_0 = (Vx + Vy) * speed - rot * speed;  // 左前轮
  velocity_1 = (Vx - Vy) * speed + rot * speed;  // 右前轮
  velocity_2 = (Vx - Vy) * speed - rot * speed;  // 左后轮
  velocity_3 = (Vx + Vy) * speed + rot * speed;  // 右后轮

  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}

/**
 * @brief PWM与轮子转向设置函数（标准Z字形索引）
 * @param Motor_0   标准索引0的速度（左前轮）
 * @param Motor_1   标准索引1的速度（右前轮）
 * @param Motor_2   标准索引2的速度（左后轮）
 * @param Motor_3   标准索引3的速度（右后轮）
 * @retval None
 *
 * @note 引脚数组已按标准Z字形索引排列，直接使用索引访问
 *       索引0 → motorpwmPin[0]=9  → 左前轮
 *       索引1 → motorpwmPin[1]=10 → 右前轮
 *       索引2 → motorpwmPin[2]=6  → 左后轮
 *       索引3 → motorpwmPin[3]=11 → 右后轮
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3)
{
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};  // 标准索引的速度值

  /**
   * 标准Z字形定义下的基准方向数组
   * 前进时：左侧轮(左前、左后)方向位=0，右侧轮(右前、右后)方向位=1
   *
   * direction[i] 对应标准索引i的基准方向：
   *   direction[0]=0 : 左前轮，方向位=0
   *   direction[1]=1 : 右前轮，方向位=1
   *   direction[2]=0 : 左后轮，方向位=0
   *   direction[3]=1 : 右后轮，方向位=1
   */
  bool direction[4] = { 0, 1, 0, 1 };

  // 遍历标准索引，直接控制对应引脚
  for(uint8_t i = 0; i < 4; ++i)
  {
    // 根据速度正负决定最终方向
    bool final_direction = direction[i];
    if(motors[i] < 0) final_direction = !final_direction;

    // 计算PWM占空比（取绝对值）
    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = abs(motors[i]);

    // 写入引脚（索引直接对应引脚数组）
    digitalWrite(motordirectionPin[i], final_direction);
    PWM_Out(motorpwmPin[i], pwm_set[i]);
  }
}

/* 模拟PWM输出 */
void PWM_Out(uint8_t PWM_Pin, int8_t DutyCycle)
{
  uint32_t currentTime_us = micros();
  int highTime = (period/100) * DutyCycle;
  int lowTime = period - highTime;

  if ((currentTime_us - previousTime_us) <= highTime)
  {
    digitalWrite(PWM_Pin, HIGH);
  }
  else digitalWrite(PWM_Pin, LOW);
  if (currentTime_us - previousTime_us >= period)
  {
    previousTime_us = currentTime_us;
  }
}
