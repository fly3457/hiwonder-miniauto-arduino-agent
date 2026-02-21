/**
 * @file motor_speed_visual_test.ino
 * @brief 电机转速视觉测试脚本
 * @description 
 *   纯视觉方法测量4个电机实际转速差异
 *   不需要编码器、IMU等额外硬件
 * 
 * @usage
 *   1. 将小车悬空（轮子离地，可以放在盒子/书上）
 *   2. 在每个轮子的同一位置贴标记（胶带/贴纸/笔迹）
 *   3. 打开串口监视器（115200波特率）
 *   4. 根据提示观察轮子转动情况
 *   5. 记录每个轮子的实际表现
 * 
 * @test_modes
 *   - 单轮测试: 逐个测试每个电机，精确测量
 *   - 四轮同步测试: 4个轮子同时转，对比观察
 *   - 平移模拟测试: 模拟左右平移，观察是否偏转
 */

#ifndef MOTOR_SPEED_VISUAL_TEST_H
#define MOTOR_SPEED_VISUAL_TEST_H

// ==================== 测试配置 ====================

#define TEST_PWM_SPEED        50      // 测试速度 (0-100)
#define TEST_DURATION_MS      5000    // 每次测试持续时间 (毫秒)
#define TEST_PAUSE_MS         2000    // 测试间隔停顿时间 (毫秒)

// 串口命令定义
#define CMD_VISUAL_TEST_START   'V'     // 启动视觉测试模式
#define CMD_SINGLE_WHEEL_TEST   'S'     // 单轮测试
#define CMD_SYNC_TEST           'Y'     // 四轮同步测试
#define CMD_TRANSLATION_TEST    'T'     // 平移模拟测试
#define CMD_STOP_TEST           'X'     // 停止测试

// 测试状态
enum VisualTestState {
  VISUAL_TEST_IDLE,           // 空闲
  VISUAL_TEST_SINGLE_WHEEL,   // 单轮测试中
  VISUAL_TEST_SYNC,           // 四轮同步测试中
  VISUAL_TEST_TRANSLATION     // 平移模拟测试中
};

static VisualTestState visual_test_state = VISUAL_TEST_IDLE;
static uint8_t current_test_wheel = 0;        // 当前测试的轮子 (0-3)
static unsigned long test_start_time = 0;     // 测试开始时间
static unsigned long last_beep_time = 0;      // 上次提示音时间
static bool test_beep_state = false;          // 提示音状态

// ==================== 函数声明 ====================

void VisualTest_Init(void);
void VisualTest_Task(void);
void VisualTest_StartSingleWheel(void);
void VisualTest_StartSync(void);
void VisualTest_StartTranslation(void);
void VisualTest_Stop(void);
void VisualTest_PrintBanner(void);
void VisualTest_Beep(unsigned int frequency, unsigned int duration);

// ==================== 实现 ====================

/**
 * @brief 初始化视觉测试模块
 */
void VisualTest_Init(void) {
  visual_test_state = VISUAL_TEST_IDLE;
  current_test_wheel = 0;
  Serial.println("\n========================================");
  Serial.println("  电机转速视觉测试模块已加载");
  Serial.println("========================================");
  Serial.println("可用命令:");
  Serial.println("  V - 显示测试菜单");
  Serial.println("  S - 单轮测试（逐个测试4个电机）");
  Serial.println("  Y - 四轮同步测试（对比观察）");
  Serial.println("  T - 平移模拟测试（观察偏转）");
  Serial.println("  X - 停止测试");
  Serial.println("========================================\n");
}

/**
 * @brief 主测试任务（在loop中调用）
 */
void VisualTest_Task(void) {
  unsigned long current_time = millis();
  
  switch (visual_test_state) {
    
    // -------------------- 单轮测试 --------------------
    case VISUAL_TEST_SINGLE_WHEEL: {
      // 检查是否到达测试时间
      if (current_time - test_start_time >= TEST_DURATION_MS) {
        // 停止当前轮子
        Motors_Set(0, 0, 0, 0);
        
        Serial.println("\n>>> 测试结束 <<<");
        Serial.println("请记录：轮子 " + String(current_test_wheel) + " 的转动情况");
        Serial.println("标记位置变化了多少？\n");
        
        // 停顿一下让用户记录
        delay(TEST_PAUSE_MS);
        
        // 切换到下一个轮子
        current_test_wheel++;
        
        if (current_test_wheel < 4) {
          // 开始测试下一个轮子
          Serial.println("========================================");
          Serial.println("准备测试轮子 " + String(current_test_wheel) + 
                        " (" + String(current_test_wheel == 0 ? "左前" : 
                                      current_test_wheel == 1 ? "右前" : 
                                      current_test_wheel == 2 ? "左后" : "右后") + ")");
          Serial.println("3秒后开始...");
          Serial.println("========================================");
          
          // 3秒倒计时
          for (int i = 3; i > 0; i--) {
            Serial.println(String(i) + "...");
            VisualTest_Beep(1000, 100);
            delay(1000);
          }
          
          Serial.println("开始！观察轮子 " + String(current_test_wheel) + " 的转动");
          Serial.println("（持续 " + String(TEST_DURATION_MS / 1000) + " 秒）\n");
          
          // 启动当前轮子
          int8_t wheel_speeds[4] = {0, 0, 0, 0};
          wheel_speeds[current_test_wheel] = TEST_PWM_SPEED;
          Motors_Set(wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
          
          test_start_time = current_time;
        } else {
          // 所有轮子测试完成
          Serial.println("\n========================================");
          Serial.println("  所有轮子测试完成！");
          Serial.println("========================================");
          Serial.println("请对比4个轮子的记录:");
          Serial.println("  - 轮子0 (左前): ___ 圈/半圈");
          Serial.println("  - 轮子1 (右前): ___ 圈/半圈");
          Serial.println("  - 轮子2 (左后): ___ 圈/半圈");
          Serial.println("  - 轮子3 (右后): ___ 圈/半圈");
          Serial.println("\n如果数值差异超过10%，建议调整motor_calibration系数");
          Serial.println("========================================\n");
          
          visual_test_state = VISUAL_TEST_IDLE;
          current_test_wheel = 0;
        }
      } else {
        // 测试进行中，每秒提示一次
        if (current_time - last_beep_time >= 1000) {
          last_beep_time = current_time;
          VisualTest_Beep(500, 50);  // 短促提示音
          Serial.println("...测试中 (" + String((current_time - test_start_time) / 1000 + 1) + "/" + 
                        String(TEST_DURATION_MS / 1000) + "秒)");
        }
      }
      break;
    }
    
    // -------------------- 四轮同步测试 --------------------
    case VISUAL_TEST_SYNC: {
      if (current_time - test_start_time >= TEST_DURATION_MS) {
        Motors_Set(0, 0, 0, 0);
        
        Serial.println("\n========================================");
        Serial.println("  四轮同步测试结束！");
        Serial.println("========================================");
        Serial.println("请观察4个轮子的标记位置:");
        Serial.println("  - 所有轮子的标记是否对齐？");
        Serial.println("  - 哪个轮子转得最快/最慢？");
        Serial.println("\n如果有明显差异，记下轮子的编号（0=左前, 1=右前, 2=左后, 3=右后）");
        Serial.println("========================================\n");
        
        visual_test_state = VISUAL_TEST_IDLE;
      } else {
        if (current_time - last_beep_time >= 1000) {
          last_beep_time = current_time;
          VisualTest_Beep(500, 50);
        }
      }
      break;
    }
    
    // -------------------- 平移模拟测试 --------------------
    case VISUAL_TEST_TRANSLATION: {
      if (current_time - test_start_time >= TEST_DURATION_MS) {
        Motors_Set(0, 0, 0, 0);
        
        Serial.println("\n========================================");
        Serial.println("  平移模拟测试结束！");
        Serial.println("========================================");
        Serial.println("请观察:");
        Serial.println("  1. 小车是否水平移动？（还是有偏转）");
        Serial.println("  2. 如果有偏转，是顺时针还是逆时针？");
        Serial.println("\n理论值: 纯左移/右移时不应该有偏转");
        Serial.println("如果有偏转，说明电机速度不匹配");
        Serial.println("========================================\n");
        
        visual_test_state = VISUAL_TEST_IDLE;
      } else {
        if (current_time - last_beep_time >= 1000) {
          last_beep_time = current_time;
          VisualTest_Beep(500, 50);
        }
      }
      break;
    }
    
    case VISUAL_TEST_IDLE:
    default:
      // 空闲状态，不做任何事
      break;
  }
}

/**
 * @brief 开始单轮测试
 */
void VisualTest_StartSingleWheel(void) {
  if (visual_test_state != VISUAL_TEST_IDLE) {
    Serial.println("[警告] 当前正在测试中，请先按 X 停止");
    return;
  }
  
  current_test_wheel = 0;
  visual_test_state = VISUAL_TEST_SINGLE_WHEEL;
  
  Serial.println("\n========================================");
  Serial.println("  单轮视觉测试");
  Serial.println("========================================");
  Serial.println("测试步骤:");
  Serial.println("1. 将小车悬空（轮子离地）");
  Serial.println("2. 在每个轮子的同一位置贴上标记");
  Serial.println("3. 观察每个轮子转动 " + String(TEST_DURATION_MS / 1000) + " 秒");
  Serial.println("4. 记录每个轮子的转动圈数或角度");
  Serial.println("========================================");
  Serial.println("准备测试轮子 0 (左前)");
  Serial.println("3秒后开始...");
  Serial.println("========================================");
  
  // 3秒倒计时
  for (int i = 3; i > 0; i--) {
    Serial.println(String(i) + "...");
    VisualTest_Beep(1000, 100);
    delay(1000);
  }
  
  Serial.println("开始！观察轮子 0 (左前) 的转动");
  Serial.println("（持续 " + String(TEST_DURATION_MS / 1000) + " 秒）\n");
  
  // 启动第一个轮子
  Motors_Set(TEST_PWM_SPEED, 0, 0, 0);
  test_start_time = millis();
  last_beep_time = test_start_time;
}

/**
 * @brief 开始四轮同步测试
 */
void VisualTest_StartSync(void) {
  if (visual_test_state != VISUAL_TEST_IDLE) {
    Serial.println("[警告] 当前正在测试中，请先按 X 停止");
    return;
  }
  
  visual_test_state = VISUAL_TEST_SYNC;
  
  Serial.println("\n========================================");
  Serial.println("  四轮同步视觉测试");
  Serial.println("========================================");
  Serial.println("测试步骤:");
  Serial.println("1. 将小车悬空");
  Serial.println("2. 确保4个轮子的标记都在同一相对位置");
  Serial.println("3. 同时观察4个轮子的转动");
  Serial.println("4. 看哪个轮子转得快/慢");
  Serial.println("========================================");
  Serial.println("3秒后开始...");
  Serial.println("========================================");
  
  for (int i = 3; i > 0; i--) {
    Serial.println(String(i) + "...");
    VisualTest_Beep(1000, 100);
    delay(1000);
  }
  
  Serial.println("开始！同时观察4个轮子");
  Serial.println("（持续 " + String(TEST_DURATION_MS / 1000) + " 秒）\n");
  
  // 4个轮子同时以相同速度转动
  Motors_Set(TEST_PWM_SPEED, TEST_PWM_SPEED, TEST_PWM_SPEED, TEST_PWM_SPEED);
  test_start_time = millis();
  last_beep_time = test_start_time;
}

/**
 * @brief 开始平移模拟测试
 */
void VisualTest_StartTranslation(void) {
  if (visual_test_state != VISUAL_TEST_IDLE) {
    Serial.println("[警告] 当前正在测试中，请先按 X 停止");
    return;
  }
  
  visual_test_state = VISUAL_TEST_TRANSLATION;
  
  Serial.println("\n========================================");
  Serial.println("  平移模拟视觉测试");
  Serial.println("========================================");
  Serial.println("测试步骤:");
  Serial.println("1. 将小车放在光滑的平面上（桌面/地板）");
  Serial.println("2. 观察小车是水平移动还是有偏转");
  Serial.println("3. 记录偏转方向和角度");
  Serial.println("========================================");
  Serial.println("选择测试模式:");
  Serial.println("  L - 左移测试");
  Serial.println("  R - 右移测试");
  Serial.println("========================================");
  
  // 等待用户选择方向
  bool selected = false;
  bool is_left = true;
  unsigned long wait_start = millis();
  
  while (!selected && (millis() - wait_start < 10000)) {  // 最多等10秒
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'L' || cmd == 'l') {
        is_left = true;
        selected = true;
      } else if (cmd == 'R' || cmd == 'r') {
        is_left = false;
        selected = true;
      }
    }
  }
  
  if (!selected) {
    Serial.println("超时，默认进行左移测试");
    is_left = true;
  }
  
  Serial.println("\n准备进行" + String(is_left ? "左移" : "右移") + "测试");
  Serial.println("3秒后开始...");
  
  for (int i = 3; i > 0; i--) {
    Serial.println(String(i) + "...");
    VisualTest_Beep(1000, 100);
    delay(1000);
  }
  
  Serial.println("开始！观察小车移动");
  Serial.println("是否有偏转？偏转方向是？\n");
  
  // 设置平移速度
  // 左移: 左前(+V), 右前(-V), 左后(-V), 右后(+V)
  // 右移: 左前(-V), 右前(+V), 左后(+V), 右后(-V)
  if (is_left) {
    Motors_Set(TEST_PWM_SPEED, -TEST_PWM_SPEED, -TEST_PWM_SPEED, TEST_PWM_SPEED);
  } else {
    Motors_Set(-TEST_PWM_SPEED, TEST_PWM_SPEED, TEST_PWM_SPEED, -TEST_PWM_SPEED);
  }
  
  test_start_time = millis();
  last_beep_time = test_start_time;
}

/**
 * @brief 停止测试
 */
void VisualTest_Stop(void) {
  Motors_Set(0, 0, 0, 0);
  visual_test_state = VISUAL_TEST_IDLE;
  current_test_wheel = 0;
  Serial.println("\n[测试已停止]\n");
}

/**
 * @brief 打印测试菜单
 */
void VisualTest_PrintBanner(void) {
  Serial.println("\n========================================");
  Serial.println("  电机转速视觉测试菜单");
  Serial.println("========================================");
  Serial.println("测试命令:");
  Serial.println("  S - 单轮测试（推荐第一次使用）");
  Serial.println("      逐个测试4个电机，精确对比");
  Serial.println("");
  Serial.println("  Y - 四轮同步测试");
  Serial.println("      4个轮子同时转，直观对比");
  Serial.println("");
  Serial.println("  T - 平移模拟测试");
  Serial.println("      模拟左右平移，观察是否偏转");
  Serial.println("");
  Serial.println("  X - 停止当前测试");
  Serial.println("========================================");
  Serial.println("准备步骤:");
  Serial.println("1. 将小车悬空（单轮测试/同步测试）");
  Serial.println("   或放在光滑平面（平移测试）");
  Serial.println("2. 在每个轮子同一位置贴标记");
  Serial.println("3. 发送命令开始测试");
  Serial.println("========================================\n");
}

/**
 * @brief 提示音（如果有蜂鸣器的话）
 */
void VisualTest_Beep(unsigned int frequency, unsigned int duration) {
  // 使用主代码中的buzzerPin
  extern const uint8_t buzzerPin;
  tone(buzzerPin, frequency);
  delay(duration);
  noTone(buzzerPin);
}

#endif // MOTOR_SPEED_VISUAL_TEST_H
