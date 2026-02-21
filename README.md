# MiniAuto 麦轮小车驱动程序

## 项目说明

这是配合 **Hiwonder 幻尔科技** 购买的 **miniAuto 麦克纳姆轮小车** 使用的驱动端程序。

**注意**: 本项目是自学单片机程序的临时产物,尚不成熟,仅供参考学习。

## 硬件架构

由于小车自带的蓝牙模块损坏,采用以下改造方案:

```
手机 APP (Flutter BLE 主机)
    ↓ BLE 蓝牙 (FFE0/FFE1)
ESP32S3-Cam 模块 (BLE 从机 + I2C 从机)
    ↓ I2C 总线 (地址 0x52, 100kHz)
Arduino Uno 模块 (I2C 主机)
    ↓ PWM + 方向控制
麦克纳姆轮小车 (4个电机)
```

**架构特点**:
- ESP32 作为 BLE 转发桥,将手机指令转发给 Arduino
- Arduino 保留原始驱动程序的通讯指令 (字符串协议: `X|data|$`)
- I2C 主机轮询模式: Arduino 主动轮询 ESP32 获取命令 (每 10ms)
- 传感器主动上报: Arduino 主动上报数据给 ESP32 (每 200ms),ESP32 立即 BLE notify 给手机

---

## 项目结构

```
hiwonder-miniauto-arduino-agent/
├── esp32_command_hub/           # ESP32 BLE 转发固件
│   └── esp32_command_hub.ino    # 主程序
│
└── arduino_motion_core/         # Arduino 麦轮驱动固件
    ├── arduino_motion_core.ino  # 主程序
    ├── Ultrasound.cpp           # 超声波传感器库
    └── Ultrasound.h
```

---

## esp32_command_hub (BLE 转发固件)

### 功能说明

ESP32S3-Cam 作为 BLE 从机和 I2C 从机,桥接手机 APP 和 Arduino:

**BLE 端**:
- 模拟 DXBT24-5.0 蓝牙模块 (UUID: FFE0/FFE1)
- 接收手机 APP 发送的控制指令
- 主动推送传感器数据给手机 (notify)

**I2C 端**:
- 地址 `0x52` (从机模式)
- 寄存器 `0x10`: 提供 BLE 命令给 Arduino 读取
- 寄存器 `0x11`: 接收 Arduino 上报的传感器数据

### 关键优化

1. **环形命令队列 (FIFO, 16条容量)**
   - 解决快速连续命令丢失问题
   - 先进先出,保证命令顺序

2. **停止指令优先级队列 (4条容量)**
   - `A|8|$` 停止指令高优先级处理
   - 到达时清空普通队列,防止停止后继续运动
   - 自动去重,避免重复停止指令

3. **传感器数据主动推送**
   - 收到 Arduino 上报的传感器数据后,立即 BLE notify 给手机
   - 延迟 < 1ms,取代原方案的 200ms 轮询延迟

### 硬件连接

```
ESP32S3-Cam          Arduino Uno
IO47 (SDA)    ----   A4 (SDA)
IO48 (SCL)    ----   A5 (SCL)
GND           ----   GND
5V            ----   5V
```

**注意**: ESP32 板载排针的丝印可能与实际引脚相反,按实际功能连接即可。

### 烧录配置

- **开发板**: ESP32S3 Dev Module
- **USB CDC On Boot**: Enabled
- **串口波特率**: 230400
- **调试开关**: `DEBUG_ENABLE` (0=关闭, 1=开启)

---

## arduino_motion_core (麦轮驱动固件)

### 功能说明

Arduino Uno 作为 I2C 主机,负责运动控制和传感器采集:

**运动控制**:
- 标准 Z 字形麦克纳姆轮运动学逆解 (ROS 兼容)
- 8 方向移动 + 原地旋转
- 软件 PWM 驱动 (每个电机独立计时器)

**传感器采集**:
- 超声波测距 (每 50ms 采样)
- 电压监测 (每 200ms 采样)
- 数据主动上报给 ESP32 (每 200ms)

**通信协议**: 字符串协议 `X|data1|data2|...|$`

### 工作原理

#### 1. 命令获取 (非阻塞)

```cpp
pollESP32Command();  // 每 10ms 轮询 ESP32 获取 BLE 命令
// I2C 读取寄存器 0x10,获取命令字符串
// 解析到 i2c_rec_data[] 缓冲区
```

#### 2. 任务调度

```cpp
Task_Dispatcher();  // 统一调度所有任务
// 根据命令类型 (A/B/C/F/G) 设置任务模式 g_mode
// 执行对应任务: 运动控制/RGB灯/速度调节/避障/电机测试
```

#### 3. 运动学解算

```cpp
Velocity_Controller(angle, velocity, rot);
// 标准 Z 字形公式:
// V_左前 = Vx + Vy + Vθ
// V_右前 = Vx - Vy - Vθ
// V_左后 = Vx - Vy + Vθ
// V_右后 = Vx + Vy - Vθ
```

#### 4. 传感器上报 (非阻塞)

```cpp
sampleUltrasound();         // 每 50ms 采样超声波
sampleVoltage();            // 每 200ms 采样电压
reportSensorDataToESP32();  // 每 200ms 上报给 ESP32
// I2C 写入寄存器 0x11: [distance_h][distance_l][voltage_h][voltage_l]
```

### 标准 Z 字形索引定义

```
     车头方向(0°)
         ↑
   0左前 ─── 1右前
     |         |
   2左后 ─── 3右后
```

| 索引 | 位置 | PWM引脚 | 方向引脚 |
|------|------|---------|---------|
| 0 | 左前轮 | 9 | 8 |
| 1 | 右前轮 | 10 | 12 |
| 2 | 左后轮 | 6 | 7 |
| 3 | 右后轮 | 11 | 13 |

### 烧录配置

- **开发板**: Arduino Uno
- **串口波特率**: 115200
- **调试开关**:
  - `ULTRASOUND_TIMING_DEBUG`: 超声波耗时监控
  - `LOOP_TIMING_DEBUG`: Loop 循环性能监控

---

## 通信协议

### BLE 连接参数

| 参数 | 值 |
|------|-----|
| 服务 UUID | `0000FFE0-0000-1000-8000-00805F9B34FB` |
| 特征 UUID | `0000FFE1-0000-1000-8000-00805F9B34FB` |
| MTU | 512 字节 |
| 连接间隔 | 7.5-15ms |

### 指令格式

**协议格式**: `X|data1|data2|...|$`

| 命令 | 格式 | 功能 |
|------|------|------|
| A | `A\|state\|$` | 运动控制 (state: 0-10) |
| B | `B\|r\|g\|b\|$` | RGB 灯光 (r/g/b: 0-255) |
| C | `C\|speed\|$` | 速度控制 (speed: 20-100) |
| D | `D\|$` | 请求传感器数据 |
| G | `G\|motorId\|speed\|$` | 电机测试 (motorId: 0-3, speed: -100到100) |

**示例**:
```
A|2|$           → 前进
A|8|$           → 停止 (清空队列,高优先级)
B|255|0|0|$     → 红色 RGB 灯
C|50|$          → 速度设置为 50
G|0|50|$        → 左前轮正转,速度 50
```

### 数据返回格式

```
$distance,voltage$
```

- `distance`: 超声波距离 (mm)
- `voltage`: 电池电压 (mV)

**示例**: `$150,8200$` (距离 150mm, 电压 8.2V)

---

## 编译和烧录

### ESP32 固件

1. 打开 Arduino IDE
2. 安装 ESP32 开发板支持 (2.x 版本)
3. 选择开发板: **ESP32S3 Dev Module**
4. 配置:
   - USB CDC On Boot: **Enabled**
   - Upload Speed: 921600
5. 打开 `esp32_command_hub/esp32_command_hub.ino`
6. 编译并上传

### Arduino 固件

1. 打开 Arduino IDE
2. 选择开发板: **Arduino Uno**
3. 选择端口: COM 口
4. 打开 `arduino_motion_core/arduino_motion_core.ino`
5. 编译并上传

---

## 测试步骤

### 1. 硬件连接

- 连接 ESP32 和 Arduino (I2C: SDA/SCL/GND/5V)
- Arduino 连接电机驱动板和传感器
- ESP32 通过 USB 供电 (或使用独立电源)

### 2. 烧录固件

- 先烧录 Arduino 固件
- 再烧录 ESP32 固件

### 3. 串口监视器测试

**ESP32 端**:
```
[初始化] ✓ I2C 从机初始化完成
[初始化] ✓ BLE 设备已启动！
[提示] 请使用 Android 端扫描并连接此设备
```

**Arduino 端**:
```
[I2C] Arduino 初始化为 I2C 主机
[I2C扫描] 找到设备: 0x52 (ESP32-S3)
[启动] Arduino UNO 已就绪
```

### 4. 手机 APP 测试

- 打开 Android 控制 APP
- 扫描并连接 "MiniAuto-ESP32"
- 测试运动控制、灯光控制、速度调节

---

## 优化总结

### 通信可靠性优化

1. **停止指令优先级**: 到达时清空队列,连续发送 3 次
2. **命令队列扩容**: 从 8 条增至 16 条 (FIFO)
3. **I2C 数据量优化**: 从 32 字节减至 16 字节

### 延迟优化

1. **轮询频率提升**: 从 50ms 降至 10ms (提速 5 倍)
2. **传感器主动上报**: 延迟从 200ms 降至 < 1ms
3. **BLE 连接参数优化**: 间隔 7.5-15ms,延迟 0

### 软件 PWM 优化

1. **独立计时器**: 每个电机独立 PWM 周期,速度误差 < 2%
2. **解决 Timer1 冲突**: 引脚 9/10 与 Servo 库冲突,使用软件 PWM

---

## 相关文档

- **手机控制端**: [hiwonder-miniauto-android-controller](https://github.com/yourusername/hiwonder-miniauto-android-controller)
- **ESP32 调试文档**: `esp32_command_hub/README.md`

---

## 更新日志

### v2.1 (2026-01-11)

- 统一停止指令协议: `A|8|$` 同时停止移动和旋转
- 停止指令优先级队列: 清空普通队列 + 自动去重
- 传感器主动上报: Arduino 主动推送,ESP32 立即 notify
- 软件 PWM 修复: 独立计时器,解决速度误差

### v2.0 (2026-01-01)

- 标准 Z 字形麦克纳姆轮运动学定义 (ROS 兼容)
- ESP32 I2C 桥接架构
- 环形命令队列 (FIFO, 16 条)

### v1.0 (2025-12-28)

- 首次提交: 基于官方 app_control.ino
- 电机测试功能 (G 命令)

---

## 许可证

本项目用于学习和研究目的。
