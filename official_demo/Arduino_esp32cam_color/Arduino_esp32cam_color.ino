#include "hw_esp32cam_ctl.h" //导入ESP32Cam通讯库

#define COLOR_1   1
#define COLOR_2   2

//ESP32Cam通讯对象
HW_ESP32Cam hw_cam;

void espcam_task(void); /* esp32cam通讯任务 */

void setup() {
  Serial.begin(115200);
  // 设置串行端口读取数据的超时时间
  Serial.setTimeout(500);

  hw_cam.begin(); //初始化与ESP32Cam通讯接口

  delay(500);
  Serial.println("start");

  // 添加 I2C 扫描功能
  Serial.println("\n[I2C扫描] 正在扫描I2C总线...");
  bool found = false;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  找到设备: 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x52) {
        Serial.print(" (ESP32-S3) ✓");
      }
      Serial.println();
      found = true;
    }
  }
  if (!found) {
    Serial.println("  ⚠ 警告: 未找到任何I2C设备!");
  }
  Serial.println("[I2C扫描] 扫描完成\n");
}

void loop() {
  // esp32cam通讯任务
  espcam_task();
}

void espcam_task(void)
{
  static uint32_t last_tick = 0;
  int color = 0;
  
  if (millis() - last_tick < 100) {
    return;
  }
  last_tick = millis();
  
  color = hw_cam.colorDetect(); //获取颜色
  if(color == COLOR_1)
  {
    Serial.println("COLOR 1");
  }else if(color == COLOR_2)
  {
    Serial.println("COLOR 2");
  }
}
