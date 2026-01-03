/**
 * LED 闪烁测试
 * 用途：确认程序是否成功烧录到 ESP32-S3
 *
 * 测试方法：上传后观察板载 LED 是否闪烁
 */

void setup() {
  // 尝试多个可能的 LED 引脚
  pinMode(2, OUTPUT);   // GPIO 2
  pinMode(4, OUTPUT);   // GPIO 4
  pinMode(13, OUTPUT);  // GPIO 13
}

void loop() {
  // 所有可能的 LED 引脚都闪烁
  digitalWrite(2, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(13, HIGH);
  delay(500);

  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  digitalWrite(13, LOW);
  delay(500);
}
