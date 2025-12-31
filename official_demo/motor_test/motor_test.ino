/**
 * @file motor_test.ino
 * @brief 电机驱动实验
 * @author Anonymity(Anonymity@hiwonder.com)
 * @version V1.0
 * @date 2024-04-22
 *
 * @copyright Copyright (c) 2023
 *
 * [0] --|||--[1]       数字对应motorpwmPin[4]和motordirectionPin[4]索引号
 *  |          |        ^逆时针
 *  |          |        |
 *  |          |        |
 *  |          |        |
 * [3] -------[2]
 */

#include <Arduino.h>

const static uint8_t pwm_min = 2;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11};            ///< 控制轮子速度引脚
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};       ///< 控制轮子方向引脚

void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot,bool drift);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

void setup() {
  Serial.begin(9600);
  // 设置串行端口读取数据的超时时间
  Serial.setTimeout(500);
  Motor_Init();                     ///< 电机初始化
  Velocity_Controller(0,50,0,0);   ///< 直行
  delay(1000);
  Velocity_Controller(45,50,0,0);  ///< 左前方移动
  delay(1000);
  Velocity_Controller(90,50,0,0);  ///< 左移
  delay(1000);
  Velocity_Controller(135,50,0,0); ///< 左后方移动
  delay(1000);
  Velocity_Controller(180,50,0,0); ///< 后移
  delay(1000);
  Velocity_Controller(225,50,0,0); ///< 右后方移动
  delay(1000);
  Velocity_Controller(270,50,0,0); ///< 右移
  delay(1000);
  Velocity_Controller(315,50,0,0); ///< 右前方移动
  delay(1000);
  Velocity_Controller(0, 0,50,0);  ///< 原地左转
  delay(1000);
  Velocity_Controller(0, 0,-50,0); ///< 原地右转
  delay(1000);
  Velocity_Controller(0,100,50,1);  ///< 漂移
  delay(1000); 
  Velocity_Controller(0,0,0,0);     ///< 静止   
}

void loop() {
}

 /* 电机初始化函数 */
void Motor_Init(void){
  for(uint8_t i = 0; i < 4; i++){
    pinMode(motordirectionPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0, 0);
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
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot,bool drift) {
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;
  angle += 90;
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;///< 速度因子
  else speed = 0.5; 
  velocity /= sqrt(2);
  if (drift) {
    velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed;
    velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed;
    velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed * 2;
    velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed * 2;
  } else {
    velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
    velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
    velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
    velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  }
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}
 
/**
 * @brief PWM与轮子转向设置函数
 * @param Motor_x   作为PWM与电机转向的控制数值。根据麦克纳姆轮的运动学分析求得。
 * @retval None
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) {
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0
  for(uint8_t i; i < 4; ++i) {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);

    digitalWrite(motordirectionPin[i], direction[i]); 
    analogWrite(motorpwmPin[i], pwm_set[i]); 
  }
}


