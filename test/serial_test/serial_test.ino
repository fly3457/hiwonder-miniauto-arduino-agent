/**
 * ESP32-S3 串口测试程序
 * 用途：测试串口通信是否正常
 */

void setup() {
  // 初始化串口
  Serial.begin(115200);

  // 等待串口准备就绪（ESP32-S3 特有）
  delay(2000);

  // 发送启动信息
  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32-S3 串口测试程序");
  Serial.println("  如果看到这条信息，说明串口正常！");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  // 每秒打印一次
  Serial.println("hiwonder - 测试中...");
  delay(1000);
}
