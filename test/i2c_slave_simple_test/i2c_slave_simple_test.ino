/**
 * ESP32-S3 最简单的 I2C 从机测试
 * 功能：模拟官方 ESP32CAM 的寄存器读写协议
 * 地址：0x52
 *
 * 测试目的：验证硬件连接是否正常
 */

#include <Wire.h>

// I2C 配置
// 使用与官方程序完全相同的配置
#define I2C_SDA_PIN 47
#define I2C_SCL_PIN 48
#define I2C_SLAVE_ADDR 0x52
#define I2C_FREQUENCY 100000  // 100kHz

// 模拟寄存器数据（类似官方程序）
uint8_t register_data[2][4] = {
  {10, 20, 30, 40},  // 寄存器 0x00 的数据（颜色1）
  {50, 60, 70, 80}   // 寄存器 0x01 的数据（颜色2/人脸）
};

// 当前要读取的寄存器地址
volatile uint8_t current_register = 0xFF;

// 统计信息
volatile unsigned long receive_count = 0;
volatile unsigned long request_count = 0;

/**
 * @brief I2C 接收回调 - 主机发送数据时调用
 * @note 官方协议：主机先发送寄存器地址，然后请求读取
 */
void onReceive(int numBytes) {
  receive_count++;

  if (Wire.available()) {
    // 读取寄存器地址
    current_register = Wire.read();

    Serial.print("[I2C] 收到寄存器地址: 0x");
    if (current_register < 16) Serial.print("0");
    Serial.print(current_register, HEX);
    Serial.print(" (");
    Serial.print(receive_count);
    Serial.println(")");
  }
}

/**
 * @brief I2C 请求回调 - 主机请求数据时调用
 * @note 根据之前收到的寄存器地址，返回对应的数据
 */
void onRequest() {
  request_count++;

  // 根据寄存器地址返回数据
  if (current_register == 0x00) {
    // 返回寄存器 0x00 的数据（使用官方的 slaveWrite）
    Wire.slaveWrite(register_data[0], 4);
    Serial.print("[I2C] 发送寄存器 0x00 数据: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(register_data[0][i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  else if (current_register == 0x01) {
    // 返回寄存器 0x01 的数据（使用官方的 slaveWrite）
    Wire.slaveWrite(register_data[1], 4);
    Serial.print("[I2C] 发送寄存器 0x01 数据: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(register_data[1][i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  else {
    // 未知寄存器，返回空数据
    uint8_t dummy[4] = {0, 0, 0, 0};
    Wire.slaveWrite(dummy, 4);
    Serial.print("[I2C] 未知寄存器 0x");
    Serial.print(current_register, HEX);
    Serial.println("，返回空数据");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  ESP32-S3 简化 I2C 从机测试");
  Serial.println("  模拟官方 ESP32CAM 协议");
  Serial.println("========================================\n");

  // 启用上拉电阻
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  Serial.println("[初始化] ✓ 已启用 SDA/SCL 上拉电阻");

  // 初始化 I2C 从机 (使用官方参数顺序)
  // Wire.begin(从机地址, SDA引脚, SCL引脚, 频率)
  Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
  Serial.println("[初始化] ✓ I2C 从机初始化完成");

  // 注册回调函数
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.print("[初始化] I2C SDA: GPIO");
  Serial.println(I2C_SDA_PIN);
  Serial.print("[初始化] I2C SCL: GPIO");
  Serial.println(I2C_SCL_PIN);
  Serial.print("[初始化] I2C 从机地址: 0x");
  Serial.println(I2C_SLAVE_ADDR, HEX);

  Serial.println("\n✓✓✓ 初始化完成！");
  Serial.println("等待 Arduino 主机连接...\n");
}

void loop() {
  // 每 5 秒打印统计信息
  static unsigned long last_report = 0;
  if (millis() - last_report >= 5000) {
    last_report = millis();

    Serial.println("\n========== I2C 统计 ==========");
    Serial.print("接收次数: ");
    Serial.println(receive_count);
    Serial.print("请求次数: ");
    Serial.println(request_count);
    Serial.println("=============================\n");
  }

  delay(10);
}
