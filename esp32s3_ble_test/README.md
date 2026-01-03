# ESP32-S3-CAM BLE 测试程序

## 📋 程序说明

这是一个用于测试 ESP32-S3-CAM 蓝牙 BLE 功能的测试程序，用于替代损坏的 DXBT24-5.0 蓝牙模块。

**功能：**
- ✅ 创建 BLE Server，广播设备名称
- ✅ 接收 Android 端发送的控制命令（A/B/C/D/E/F）
- ✅ 解析并打印命令参数（串口监视器查看）
- ✅ 模拟返回电压和距离数据（D 命令）
- ✅ LED 指示灯显示连接状态
- ✅ 支持自动重连

**重要提示：**
- 这是 **测试程序**，不包含 I2C 转发功能
- 主要用于确认 ESP32-S3 的 BLE 名称和 UUID
- 验证 Android 端能否正常连接和通信

---

## 🔧 Arduino IDE 配置

### 1. 安装 ESP32 开发板支持

**首次使用需要安装 ESP32 板卡包：**

#### 方法一：通过开发板管理器安装（推荐）

1. 打开 **Arduino IDE**
2. 点击 **文件 → 首选项**
3. 在 **"附加开发板管理器网址"** 中添加：
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
4. 点击 **工具 → 开发板 → 开发板管理器**
5. 搜索 **"esp32"**
6. 安装 **"esp32 by Espressif Systems"**（选择最新稳定版，如 2.0.14 或 3.0.x）
7. 等待安装完成（可能需要 5-10 分钟）

#### 方法二：使用国内镜像（如果官方源速度慢）

在首选项中使用以下网址：
```
https://mirrors.tuna.tsinghua.edu.cn/espressif/arduino-esp32/package_esp32_index.json
```

---

### 2. 选择开发板

1. 点击 **工具 → 开发板 → ESP32 Arduino**
2. 选择 **"ESP32S3 Dev Module"**

**重要参数配置：**

| 配置项 | 设置值 | 说明 |
|--------|--------|------|
| **Upload Speed** | 921600 | 上传速度（可降低为 115200） |
| **USB Mode** | Hardware CDC and JTAG | USB 模式 |
| **USB CDC On Boot** | Enabled | 串口启用 |
| **CPU Frequency** | 240MHz | CPU 频率 |
| **Flash Mode** | QIO 80MHz | Flash 模式 |
| **Flash Size** | 4MB (32Mb) | Flash 大小 |
| **Partition Scheme** | Default 4MB with spiffs | 分区方案 |
| **Core Debug Level** | None | 调试级别（可选 Info/Debug） |
| **PSRAM** | OPI PSRAM | PSRAM 配置（ESP32-S3-CAM 有 PSRAM） |
| **Arduino Runs On** | Core 1 | Arduino 运行核心 |
| **Events Run On** | Core 1 | 事件运行核心 |

**快速配置截图位置：**
```
工具菜单下的所有选项
```

---

### 3. 选择端口

1. 用 USB 线连接 ESP32-S3-CAM 到电脑
2. 点击 **工具 → 端口**
3. 选择对应的 COM 端口（如 COM3、COM4 等）

**如何识别正确的端口：**
- Windows: 通常显示为 `COM3 (USB-SERIAL CH340)` 或 `COM4`
- 如果不确定，可以先拔掉 USB，看哪个端口消失，再插上确认

**驱动问题：**
- 如果看不到端口，可能需要安装 CH340 或 CP2102 驱动程序
- 下载地址：
  - CH340: https://www.wch.cn/downloads/CH341SER_EXE.html
  - CP2102: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

---

## 🚀 烧录步骤

### 1. 打开程序

1. 启动 **Arduino IDE**
2. 点击 **文件 → 打开**
3. 找到并打开：
   ```
   D:\project\yuncii\automini\hiwonder-miniauto-arduino-agent\esp32s3_ble_test\esp32s3_ble_test.ino
   ```

---

### 2. 编译程序

1. 点击 **验证/编译** 按钮（✓ 图标）
2. 等待编译完成

**编译输出示例：**
```
草图使用了 xxxxxx 字节 (xx%) 的程序存储空间。
全局变量使用了 xxxx 字节 (xx%) 的动态内存。
```

**常见编译错误：**

#### 错误 1：`BLEDevice.h: No such file or directory`

**原因：** 未安装 ESP32 板卡包

**解决：** 按照上面 "安装 ESP32 开发板支持" 步骤操作

#### 错误 2：`compilation error: ...`

**原因：** 未选择正确的开发板

**解决：** 确保选择了 **"ESP32S3 Dev Module"**

---

### 3. 进入下载模式（重要！）

ESP32-S3 需要手动进入下载模式才能烧录程序。

**操作步骤：**

1. **按住 BOOT 按钮**（通常标记为 BOOT 或 IO0）
2. **按一下 RST/RESET 按钮**（复位按钮）
3. **松开 RST 按钮**
4. **松开 BOOT 按钮**
5. 此时 ESP32-S3 进入下载模式

**提示：**
- 如果烧录失败，重复此步骤
- 部分开发板可能自动进入下载模式，无需手动操作

---

### 4. 上传程序

1. 点击 **上传** 按钮（→ 图标）
2. 观察下方输出窗口

**正常上传输出：**
```
正在编译...
正在连接...
Chip is ESP32-S3 (revision vX.X)
...
Writing at 0x00010000... (10%)
...
Writing at 0x000e0000... (100%)
Wrote xxxxxx bytes (xxxxxx compressed) at 0x00010000 in x.x seconds
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

**上传成功标志：**
- 输出窗口显示 `"Hard resetting via RTS pin..."`
- ESP32-S3 自动复位并开始运行程序

---

### 5. 打开串口监视器

1. 点击 **工具 → 串口监视器**（或按 Ctrl+Shift+M）
2. 设置波特率为 **115200**（右下角下拉菜单）
3. 查看输出

**正常输出示例：**
```
==================================
  ESP32-S3-CAM BLE 测试程序
  MiniAuto 小车项目
==================================

[初始化] 正在初始化 BLE...
[初始化] BLE 设备已启动！

[配置信息]
  设备名称: MiniAuto-ESP32
  服务UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
  特征UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8

[提示] 请使用 Android 端扫描并连接此设备
[提示] 或使用 nRF Connect 等 BLE 工具测试
```

---

## 📱 测试步骤

### 方法一：使用 nRF Connect（推荐）

1. **安装 nRF Connect**
   - Android: [Google Play](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp)
   - iOS: [App Store](https://apps.apple.com/app/nrf-connect/id1054362403)

2. **扫描设备**
   - 打开 nRF Connect
   - 点击 **SCAN**
   - 找到 **"MiniAuto-ESP32"**

3. **连接并查看**
   - 点击 **CONNECT**
   - 查看服务列表，找到：
     ```
     服务: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
       特征: beb5483e-36e1-4688-b7f5-ea07361b26a8
     ```
   - 确认 UUID 正确

4. **测试发送命令**
   - 点击特征的 **写入图标**（向上箭头）
   - 选择 **Text** 格式
   - 输入测试命令：`D|$`
   - 点击 **SEND**
   - 观察 ESP32 串口监视器的输出

**串口监视器应显示：**
```
[BLE] 客户端已连接
[BLE] 收到数据: D|$
[命令解析] 请求电压和距离数据
  [BLE] 发送数据: $150,7400$
```

---

### 方法二：使用修改后的 Android 应用

1. **修改 Android 端 UUID**

编辑文件：`hiwonder-miniauto-android-controller/lib/bluetooth_service.dart`

```dart
// 修改这两行
static const String SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
static const String TX_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
```

2. **重新编译并安装 Android 应用**

```bash
cd hiwonder-miniauto-android-controller
flutter build apk --release
# 安装到手机
```

3. **测试连接**
   - 打开应用
   - 扫描设备，应显示 **"MiniAuto-ESP32"**
   - 点击连接
   - 测试按钮功能（前进/后退/RGB 等）

4. **查看 ESP32 串口输出**

每次按下按钮，串口监视器应显示对应的命令解析：

```
[BLE] 收到数据: A|2|$
[命令解析] 运动控制命令
  格式: A|state|$
  状态码: 2

[BLE] 收到数据: B|255|0|0|$
[命令解析] RGB 灯光控制命令
  格式: B|r|g|b|$
  R: 255, G: 0, B: 0
```

---

## 🔍 故障排查

### 问题 1：烧录失败 - "Failed to connect to ESP32"

**可能原因：**
- 未进入下载模式
- 端口选择错误
- USB 线损坏或仅供电

**解决方案：**
1. 重新执行 "进入下载模式" 步骤
2. 更换 USB 线（确保是数据线）
3. 尝试降低上传速度（工具 → Upload Speed → 115200）

---

### 问题 2：串口监视器无输出

**可能原因：**
- 波特率设置错误
- 程序未运行
- USB CDC 未启用

**解决方案：**
1. 确认波特率为 **115200**
2. 按一下 RST 按钮重启 ESP32
3. 检查 **USB CDC On Boot** 是否设置为 **Enabled**

---

### 问题 3：Android 扫描不到设备

**可能原因：**
- BLE 未正常启动
- 蓝牙权限未授予
- ESP32 程序崩溃

**解决方案：**
1. 查看串口监视器，确认显示 "BLE 设备已启动"
2. 检查 Android 应用的蓝牙和位置权限
3. 重启 ESP32（按 RST 按钮）
4. 尝试重启手机蓝牙

---

### 问题 4：连接后立即断开

**可能原因：**
- UUID 不匹配
- BLE 库版本问题

**解决方案：**
1. 确认 Android 端 UUID 与 ESP32 程序一致
2. 使用 nRF Connect 测试，排除 Android 应用问题
3. 更新 ESP32 板卡包到最新版本

---

### 问题 5：LED 不闪烁

**可能原因：**
- LED_PIN 定义不正确
- 开发板 LED 引脚不同

**解决方案：**
1. 修改 `esp32s3_ble_test.ino` 第 25 行：
   ```cpp
   #define LED_PIN 2  // 改为您开发板的 LED 引脚（如 4、13 等）
   ```
2. 如果不确定引脚号，可以注释掉所有 LED 相关代码

---

## 📊 测试检查清单

完成以下测试后，即可确认 ESP32-S3 BLE 功能正常：

- [ ] Arduino IDE 成功编译程序
- [ ] ESP32-S3 成功烧录程序
- [ ] 串口监视器显示初始化信息
- [ ] 串口监视器显示正确的设备名称和 UUID
- [ ] nRF Connect 能扫描到 "MiniAuto-ESP32"
- [ ] nRF Connect 能成功连接设备
- [ ] nRF Connect 能看到正确的服务和特征 UUID
- [ ] nRF Connect 发送 `D|$` 命令，ESP32 返回数据
- [ ] LED 指示灯在连接时亮起/闪烁
- [ ] 断开连接后 LED 熄灭，设备重新广播

---

## 📝 下一步计划

完成测试后，您可以进行以下工作：

### 1. 确认 UUID 配置
- 记录下 ESP32 的 UUID
- 在 Android 端配置相同的 UUID

### 2. 开发完整版本
- 添加 I2C Master 功能
- 实现 ESP32 → Arduino 数据转发
- 完整的命令处理逻辑

### 3. 集成测试
- ESP32 + Arduino 联调
- 完整通信链路测试
- 实车功能验证

---

## 🛠️ 程序文件说明

```
esp32s3_ble_test/
├── esp32s3_ble_test.ino  # 主程序
└── README.md             # 本文档
```

**程序结构：**
- `setup()`: 初始化 BLE、创建服务和特征
- `loop()`: 处理连接状态、LED 指示
- `handleCommand()`: 解析并打印接收到的命令
- `MyServerCallbacks`: BLE 连接/断开回调
- `MyCharacteristicCallbacks`: 数据接收回调

---

## 📚 相关文档

- **ESP32-S3 官方文档**: https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/
- **Arduino-ESP32 库**: https://github.com/espressif/arduino-esp32
- **BLE 协议说明**: 参考项目根目录 `CLAUDE.md`

---

## ⚠️ 重要提示

1. **这是测试程序**，不包含完整的小车控制功能
2. 测试完成后，需要开发包含 I2C 转发的完整版本
3. 确保 ESP32-S3-CAM 和 Arduino Uno 共地（GND 连接）
4. 正式使用时建议添加电平转换模块（3.3V ↔ 5V）

---

**测试完成后请反馈：**
- ✅ 设备名称和 UUID 是否正确显示
- ✅ Android 端能否正常连接
- ✅ 命令收发是否正常

祝测试顺利！ 🎉
