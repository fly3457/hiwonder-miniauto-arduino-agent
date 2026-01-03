/**
 * ESP32-S3 I2C 引脚测试程序
 * 用途：测试 GPIO47/GPIO48 是否可以正常工作
 *
 * 测试方法：
 * 1. 上传此程序到 ESP32-S3
 * 2. 用万用表测量 GPIO47 和 GPIO48 的电压
 * 3. 应该看到两个引脚交替输出 高(3.3V) 和 低(0V)
 */

#define TEST_PIN_SDA 47
#define TEST_PIN_SCL 48

void setup() {
  Serial.begin(115200);

  // 等待串口准备就绪
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  ESP32-S3 I2C 引脚测试程序");
  Serial.println("========================================\n");

  // 设置引脚为输出模式
  pinMode(TEST_PIN_SDA, OUTPUT);
  pinMode(TEST_PIN_SCL, OUTPUT);

  Serial.println("[测试] GPIO47 和 GPIO48 已设置为输出模式");
  Serial.println("[测试] 将交替输出高低电平");
  Serial.println("[提示] 请用万用表测量引脚电压:");
  Serial.println("  - 高电平应该约 3.3V");
  Serial.println("  - 低电平应该约 0V\n");
}

void loop() {
  // GPIO47 输出高电平
  digitalWrite(TEST_PIN_SDA, HIGH);
  Serial.println("[GPIO47] 输出 HIGH (3.3V)");
  delay(1000);

  // GPIO47 输出低电平
  digitalWrite(TEST_PIN_SDA, LOW);
  Serial.println("[GPIO47] 输出 LOW (0V)");
  delay(1000);

  // GPIO48 输出高电平
  digitalWrite(TEST_PIN_SCL, HIGH);
  Serial.println("[GPIO48] 输出 HIGH (3.3V)");
  delay(1000);

  // GPIO48 输出低电平
  digitalWrite(TEST_PIN_SCL, LOW);
  Serial.println("[GPIO48] 输出 LOW (0V)");
  delay(1000);

  Serial.println("------------------------");
}
