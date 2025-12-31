/**
 * @file app_control.ino
 * @author Anonymity(Anonymity@hiwonder.com)
 * @brief APP遥控玩法
 * @version V1.0
 * @date 2024-04-26
 *
 * @copyright Copyright (c) 2024
 *
 * @attention main函数中不可以做任何阻塞处理！
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
static int8_t test_motor_speeds[4] = {0, 0, 0, 0}; /* 测试模式下的电机速度 */

static CRGB rgbs[1];
String rec_data[4];                 /* 接收APP的发送数据 */

const char *charArray;

/* 引脚定义 */
const static uint8_t ledPin = 2;
const static uint8_t buzzerPin = 3;
const static uint8_t servoPin = 5;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11} ;
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};

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
  uint8_t motor_id = (uint8_t)atoi(rec_data[1].c_str());  // 电机编号 0-3
  int8_t speed = (int8_t)atoi(rec_data[2].c_str());       // 速度 -100到100

  // 保存到全局数组，在loop中持续输出PWM
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
 * @brief 速度控制函数
 * @param angle   用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。
 *                取值为0~359
 * @param velocity   用于控制小车速度，取值为0~100。
 * @param rot     用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆
 *                 时针的自转速度，若小于0则有一个顺时针的自转速度。
 * @param drift   用于决定小车是否开启漂移功能，取值为0或1，若为0则开启，反之关闭。
 * @retval None
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot) 
{
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;
  angle += 90;
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;///< 速度因子
  else speed = 0.5; 
  velocity /= sqrt(2);
  velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
  velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
  velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
  velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}
 
/**
 * @brief PWM与轮子转向设置函数
 * @param Motor_x   作为PWM与电机转向的控制数值。根据麦克纳姆轮的运动学分析求得。
 * @retval None
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) 
{
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0
  for(uint8_t i; i < 4; ++i) 
  {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = abs(motors[i]);

    digitalWrite(motordirectionPin[i], direction[i]); 
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
