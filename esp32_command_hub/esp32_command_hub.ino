/**
 * ESP32-S3-CAM BLE 测试程序
 * 功能：创建 BLE Server，模拟原蓝牙模块功能
 * 用途：测试 ESP32-S3 的 BLE 功能和 UUID 配置
 *
 * 硬件：ESP32-S3-CAM 开发板
 * 开发环境：Arduino IDE 2.x
 *
 * 作者：AutoMini Project
 * 日期：2026-01-02
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>  // I2C库
// 注意：BLE2902 在新版本中已弃用，描述符会自动添加

// ==================== 调试控制 ====================
// 设置为 0 关闭调试打印(提升性能), 1 开启调试打印
#define DEBUG_ENABLE 0  // ⚠️ 发布版本设为 0

// ==================== 配置参数 ====================

// I2C 配置
// 使用4pin成品线连接,引脚顺序需匹配Arduino UNO (SCL,SDA,GND,5V)
// Arduino UNO排针: 1=A5(SCL), 2=A4(SDA), 3=GND, 4=5V
// ESP32板载排针: 1=IO48,     2=IO47,     3=GND, 4=5V
// 因此需要: IO48→SCL, IO47→SDA (与丝印相反,但功能正确)
#define I2C_SDA_PIN 47   // ESP32-S3 的 SDA 引脚 (物理位置2,连接Arduino A4)
#define I2C_SCL_PIN 48   // ESP32-S3 的 SCL 引脚 (物理位置1,连接Arduino A5)
#define I2C_SLAVE_ADDR 0x52  // ESP32-S3 的 I2C 从机地址 (官方地址,与官方ESP32CAM程序一致)
#define I2C_FREQUENCY 100000  // I2C 频率 100kHz
#define COMMAND_BUFFER_SIZE 64  // 命令缓冲区大小

// BLE 设备名称（Android 扫描时显示的名称）
#define BLE_DEVICE_NAME "MiniAuto-ESP32"

// BLE 服务 UUID（兼容 DXBT24-5.0 蓝牙模块）
#define SERVICE_UUID "0000FFE0-0000-1000-8000-00805F9B34FB"

// BLE 特征 UUID（用于数据收发，兼容 DXBT24-5.0）
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

// 串口波特率
#define SERIAL_BAUD 230400

// LED 指示灯引脚（ESP32-S3-CAM 板载 LED，可选）
#define LED_PIN 2

// ==================== 全局变量 ====================

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ==================== 环形命令队列 (FIFO) ====================
// 解决命令覆盖丢失问题,支持快速连续命令
#define CMD_QUEUE_SIZE 16  // 队列大小(可存储16条命令) - 优化: 从8增至16
#define CMD_MAX_LEN 32     // 每条命令最大长度
#define PRIORITY_QUEUE_SIZE 4  // 高优先级队列大小(存储停止指令)

struct CommandQueue {
  char buffer[CMD_QUEUE_SIZE][CMD_MAX_LEN];  // 命令存储数组
  volatile uint8_t head;   // 写指针(新命令入队位置)
  volatile uint8_t tail;   // 读指针(下次出队位置)
  volatile uint8_t count;  // 当前队列中的命令数
} cmdQueue;  // 普通命令队列

struct CommandQueue priorityQueue;  // 高优先级队列(停止指令A|8|$专用)

// 旧的单命令缓冲区(保留用于统计)
volatile uint32_t total_commands_received = 0;  // 总接收命令数
volatile uint32_t commands_dropped = 0;         // 丢弃命令数(队列满)

// I2C 寄存器模拟（类似官方ColorDetection程序）
// 当前Arduino请求的寄存器地址
volatile uint8_t current_register = 0xFF;

// I2C 统计信息
volatile unsigned long i2c_request_count = 0;  // Arduino请求次数
volatile unsigned long i2c_receive_count = 0;  // Arduino发送次数
unsigned long last_request_report = 0;  // 上次报告时间

// 传感器数据存储 (Arduino通过寄存器0x11上报)
volatile uint16_t latest_distance = 150;   // 最新超声波距离 (mm), 默认150
volatile uint16_t latest_voltage = 7400;   // 最新电压 (mV), 默认7400

// ==================== 函数前置声明 ====================

void handleCommand(String command);
void onI2CRequest();  // I2C从机请求回调
void onI2CReceive(int howMany);  // I2C从机接收回调

// ==================== BLE 回调类 ====================

// 服务器连接回调
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(LED_PIN, HIGH);  // LED 亮，表示已连接
    Serial.println("\n========================================");
    Serial.println("[BLE] ✓ 客户端已连接成功！");
    Serial.println("[提示] 现在可以发送命令进行测试");
    Serial.println("========================================\n");

    // 优化BLE连接参数以降低延迟
    // 连接间隔: 7.5ms-15ms (更快的数据传输)
    // 延迟: 0 (不跳过任何连接事件)
    // 超时: 4000ms
    BLEDevice::getServer()->updateConnParams(
      pServer->getConnId(),  // 连接ID
      6,    // minInterval: 7.5ms (单位1.25ms, 6*1.25=7.5ms)
      12,   // maxInterval: 15ms (单位1.25ms, 12*1.25=15ms)
      0,    // latency: 0 (不跳过连接事件)
      400   // timeout: 4000ms (单位10ms, 400*10=4000ms)
    );
    Serial.println("[BLE] 已优化连接参数: 间隔7.5-15ms, 延迟0");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    digitalWrite(LED_PIN, LOW);  // LED 灭，表示断开
    Serial.println("\n[BLE] ✗ 客户端已断开\n");
  }
};

// 特征值写入回调
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // 使用最兼容的方法获取数据
    uint8_t* pData = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();

    // 转换为 Arduino String
    String value = "";
    for (size_t i = 0; i < len; i++) {
      value += (char)pData[i];
    }

    if (value.length() > 0) {
      // 解析并处理命令
      handleCommand(value);
    }
  }
};

// ==================== I2C 从机功能 ====================

/**
 * @brief I2C接收回调 - Arduino主机发送数据时调用
 * @note Arduino发送两种数据:
 *       1. 寄存器地址 (0x10=命令, 0x11=传感器数据)
 *       2. 传感器数据 (寄存器0x11后跟4字节: distance高低+voltage高低)
 *
 * ⭐ P2优化: 收到Arduino主动上报的传感器数据后，立即BLE notify给Android
 * 优化前: 传感器数据存储后等待Android请求D命令 (延迟最高200ms)
 * 优化后: 收到数据立即notify (延迟<1ms，取消D命令定时发送)
 */
void onI2CReceive(int howMany) {
  i2c_receive_count = i2c_receive_count + 1;  // 避免volatile警告

  // 读取寄存器地址（第一个字节）
  if (Wire.available()) {
    uint8_t reg = Wire.read();

    // 寄存器0x11: 接收传感器数据 (4字节)
    if (reg == 0x11 && Wire.available() >= 4) {
      uint8_t dist_h = Wire.read();  // 距离高字节
      uint8_t dist_l = Wire.read();  // 距离低字节
      uint8_t volt_h = Wire.read();  // 电压高字节
      uint8_t volt_l = Wire.read();  // 电压低字节

      // 合并为16位数据
      latest_distance = (dist_h << 8) | dist_l;
      latest_voltage = (volt_h << 8) | volt_l;

      // ⭐ P2优化: 立即通过BLE通知Android端 (无需等待D命令)
      if (deviceConnected && pCharacteristic != NULL) {
        String responseData = "$" + String(latest_distance) + "," + String(latest_voltage) + "$";
        pCharacteristic->setValue(responseData.c_str());
        pCharacteristic->notify();  // 主动推送给Android
      }
    }
    // 寄存器0x10: 命令寄存器标记
    else if (reg == 0x10) {
      current_register = 0x10;  // 标记为命令寄存器，供onI2CRequest使用
    }
    // 其他寄存器
    else {
      current_register = reg;
    }
  }

  // 清空其余数据（如果有）
  while (Wire.available()) {
    Wire.read();
  }
}

/**
 * @brief I2C请求回调 - Arduino主机请求数据时调用
 * @note 根据寄存器地址返回对应数据（使用官方的slaveWrite方法）
 */
void onI2CRequest() {
  i2c_request_count = i2c_request_count + 1;

  // 寄存器 0x00 和 0x01 用于兼容Arduino_esp32cam_color程序
  if (current_register == 0x00 || current_register == 0x01) {
    // 模拟颜色检测数据：x, y, w, h (如果没有命令,返回空数据)
    uint8_t dummy_data[4] = {0, 0, 0, 0};
    Wire.slaveWrite(dummy_data, 4);
  }
  // 自定义寄存器：用于发送BLE命令给Arduino
  else if (current_register == 0x10) {
    uint8_t send_buffer[16];  // 优化: 从32字节减至16字节 (最长指令B|255|255|255|$=15字节+'\0')
    memset(send_buffer, 0, 16);  // 先清零

    // ⭐ 优先级机制：优先发送停止指令
    if (priorityQueue.count > 0) {
      // 优先发送高优先级队列中的停止指令
      memcpy(send_buffer, priorityQueue.buffer[priorityQueue.tail], min(CMD_MAX_LEN, 16));
      send_buffer[15] = '\0';  // 确保字符串结束

      // 更新优先级队列指针(出队)
      priorityQueue.tail = (priorityQueue.tail + 1) % PRIORITY_QUEUE_SIZE;
      priorityQueue.count--;
    }
    // 如果优先级队列为空，检查普通队列
    else if (cmdQueue.count > 0) {
      // 从普通队列中取出命令(FIFO)
      memcpy(send_buffer, cmdQueue.buffer[cmdQueue.tail], min(CMD_MAX_LEN, 16));
      send_buffer[15] = '\0';  // 确保字符串结束

      // 更新队列指针(出队)
      cmdQueue.tail = (cmdQueue.tail + 1) % CMD_QUEUE_SIZE;
      cmdQueue.count--;
    }
    // 如果两个队列都为空，send_buffer已经全是0，发送空数据

    // 固定发送16字节 (优化: 从32减至16，I2C传输时间减半)
    Wire.slaveWrite(send_buffer, 16);
  }
  else {
    // 未知寄存器,返回空数据
    uint8_t dummy[4] = {0, 0, 0, 0};
    Wire.slaveWrite(dummy, 4);
  }
}

/**
 * @brief 存储BLE收到的命令到环形队列 (支持停止指令优先级)
 * @param command 收到的命令字符串
 *
 * ⭐ 优化: 停止指令到达时清空普通队列 + 去重
 * 方案1: 停止指令清空普通队列 - 防止停止后继续运动
 * 方案2: 停止指令去重 - 避免多次停止占满优先队列
 */
void storeCommand(String command) {
  total_commands_received++;  // 统计总接收数

  if (command.length() >= CMD_MAX_LEN) {
    commands_dropped++;
    return;
  }

  // ⭐⭐⭐ 方案1+2: 检查是否是停止指令 A|8|$ ⭐⭐⭐
  if (command == "A|8|$") {

    // ⭐ 方案1: 清空普通队列 (停止指令意味着立即停止，之前的运动指令都应作废)
    if (cmdQueue.count > 0) {
      Serial.print("[停止指令] 清空普通队列，丢弃 ");
      Serial.print(cmdQueue.count);
      Serial.print(" 条指令 (");
      Serial.print(command);
      Serial.println(")");
    }
    cmdQueue.head = 0;
    cmdQueue.tail = 0;
    cmdQueue.count = 0;

    // ⭐ 方案2: 停止指令去重 (优先队列中只保留1条最新的停止指令)
    if (priorityQueue.count > 0) {
      // 检查最后一条是否也是相同的停止指令
      uint8_t last_index = (priorityQueue.head - 1 + PRIORITY_QUEUE_SIZE) % PRIORITY_QUEUE_SIZE;
      if (strcmp(priorityQueue.buffer[last_index], command.c_str()) == 0) {
        // 已有相同的停止指令，不重复入队
        Serial.print("[停止指令] 去重，忽略重复指令 (");
        Serial.print(command);
        Serial.println(")");
        return;
      }
    }

    // 停止指令存入高优先级队列
    if (priorityQueue.count < PRIORITY_QUEUE_SIZE) {
      // 优先级队列未满，直接入队
      memset(priorityQueue.buffer[priorityQueue.head], 0, CMD_MAX_LEN);
      command.toCharArray(priorityQueue.buffer[priorityQueue.head], CMD_MAX_LEN);
      priorityQueue.head = (priorityQueue.head + 1) % PRIORITY_QUEUE_SIZE;
      priorityQueue.count++;

      Serial.print("[停止指令] 入队成功 (");
      Serial.print(command);
      Serial.print("), 优先队列: ");
      Serial.print(priorityQueue.count);
      Serial.print("/");
      Serial.println(PRIORITY_QUEUE_SIZE);
    } else {
      // 优先级队列已满（极端情况），强制覆盖最早的停止指令
      commands_dropped++;
      priorityQueue.tail = (priorityQueue.tail + 1) % PRIORITY_QUEUE_SIZE;
      priorityQueue.count--;
      // 然后入队新的停止指令
      memset(priorityQueue.buffer[priorityQueue.head], 0, CMD_MAX_LEN);
      command.toCharArray(priorityQueue.buffer[priorityQueue.head], CMD_MAX_LEN);
      priorityQueue.head = (priorityQueue.head + 1) % PRIORITY_QUEUE_SIZE;
      priorityQueue.count++;

      Serial.println("[警告] 优先队列已满，覆盖最早的停止指令");
    }
    return;  // 停止指令不进入普通队列
  }

  // 普通命令存入普通队列
  // 如果队列已满,丢弃最早的命令(覆盖策略)
  if (cmdQueue.count >= CMD_QUEUE_SIZE) {
    commands_dropped++;
    // 移动tail指针,释放一个位置
    cmdQueue.tail = (cmdQueue.tail + 1) % CMD_QUEUE_SIZE;
    cmdQueue.count--;
  }

  // 将命令存入队列头部(入队)
  memset(cmdQueue.buffer[cmdQueue.head], 0, CMD_MAX_LEN);  // 先清零
  command.toCharArray(cmdQueue.buffer[cmdQueue.head], CMD_MAX_LEN);
  cmdQueue.head = (cmdQueue.head + 1) % CMD_QUEUE_SIZE;  // 移动head指针
  cmdQueue.count++;  // 队列计数+1
}

// ==================== 命令处理函数 ====================

void handleCommand(String command) {
  // 存储命令到I2C缓冲区,等待Arduino读取
  storeCommand(command);

  // D命令特殊处理：返回最新的传感器数据给Android端
  if (command.startsWith("D|")) {
    // 使用Arduino通过寄存器0x11上报的最新传感器数据
    String responseData = "$" + String(latest_distance) + "," + String(latest_voltage) + "$";
    pCharacteristic->setValue(responseData.c_str());
    pCharacteristic->notify();  // 通知 Android 端
  }
}

// ==================== 初始化函数 ====================

void setup() {
  // 初始化 LED 指示灯（先初始化 LED，方便调试）
  pinMode(LED_PIN, OUTPUT);

  // LED 快速闪烁 5 次表示程序开始运行
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  // 初始化串口
  Serial.begin(SERIAL_BAUD);

  // 等待 USB CDC 准备就绪（ESP32-S3 特有）
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 5000)) {
    // 等待时让 LED 慢闪，表示在等待串口
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  digitalWrite(LED_PIN, LOW);
  delay(500);

  // 发送大量换行，确保串口监视器能看到
  for (int i = 0; i < 10; i++) {
    Serial.println();
  }

  Serial.println(">>>>>>>>>> 串口测试开始 <<<<<<<<<<");
  Serial.println(">>>>>>>>>> 串口测试开始 <<<<<<<<<<");
  Serial.println(">>>>>>>>>> 串口测试开始 <<<<<<<<<<");
  Serial.println();

  Serial.println("\n==================================");
  Serial.println("  ESP32-S3-CAM BLE I2C桥接程序");
  Serial.println("  MiniAuto 小车项目");
  Serial.println("==================================\n");

  // 初始化命令队列（清零，防止随机数据）
  memset(&cmdQueue, 0, sizeof(cmdQueue));
  cmdQueue.head = 0;
  cmdQueue.tail = 0;
  cmdQueue.count = 0;

  // 初始化优先级队列
  memset(&priorityQueue, 0, sizeof(priorityQueue));
  priorityQueue.head = 0;
  priorityQueue.tail = 0;
  priorityQueue.count = 0;

  total_commands_received = 0;
  commands_dropped = 0;
  Serial.println("[初始化] ✓ 命令队列已初始化 (环形FIFO,容量16条)");
  Serial.println("[初始化] ✓ 优先级队列已初始化 (停止指令专用,容量4条)");

  // 初始化 I2C (从机模式)
  Serial.println("[初始化] 正在初始化 I2C 从机...");

  // 启用内部上拉电阻（I2C 总线需要上拉）
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  Serial.println("  ✓ 已启用 SDA/SCL 内部上拉电阻");

  // ESP32 I2C从机初始化（使用官方参数顺序）
  // Wire.begin(从机地址, SDA引脚, SCL引脚, 频率)
  Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
  Serial.println("  ✓ I2C 从机初始化完成");

  Wire.onRequest(onI2CRequest);  // 注册请求回调
  Wire.onReceive(onI2CReceive);  // 注册接收回调

  Serial.print("  I2C SDA: GPIO");
  Serial.println(I2C_SDA_PIN);
  Serial.print("  I2C SCL: GPIO");
  Serial.println(I2C_SCL_PIN);
  Serial.print("  ESP32 从机地址: 0x");
  Serial.println(I2C_SLAVE_ADDR, HEX);

  // 验证I2C从机地址
  Serial.println("\n[I2C验证] 从机地址确认:");
  Serial.print("  十六进制: 0x");
  if (I2C_SLAVE_ADDR < 16) Serial.print("0");
  Serial.println(I2C_SLAVE_ADDR, HEX);
  Serial.print("  十进制: ");
  Serial.println(I2C_SLAVE_ADDR, DEC);
  Serial.print("  二进制: 0b");
  Serial.println(I2C_SLAVE_ADDR, BIN);

  Serial.println("[初始化] ✓ I2C 从机初始化完成");
  Serial.println("  等待 Arduino 主机轮询...\n");

  // 初始化 BLE 设备
  Serial.println("[初始化] 正在初始化 BLE...");
  BLEDevice::init(BLE_DEVICE_NAME);

  // 设置更大的MTU以提高数据传输效率
  BLEDevice::setMTU(512);  // 增大MTU(默认23字节)
  Serial.println("[初始化] BLE MTU已设置为512字节");

  // 创建 BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 创建 BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 创建 BLE Characteristic（支持读/写/通知）
  // 添加 WRITE_NR (Write No Response) 以确保兼容性
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // 注意：新版本 ESP32 BLE 库会自动添加 2902 描述符，无需手动添加
  // pCharacteristic->addDescriptor(new BLE2902());  // 已弃用

  // 设置写入回调
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  Serial.println("[初始化] 特征回调已设置");

  // 启动服务
  pService->start();

  // 启动广播
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // 帮助 iPhone 连接
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[初始化] BLE 设备已启动！");
  Serial.println("\n[配置信息]");
  Serial.print("  设备名称: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("  服务UUID: ");
  Serial.println(SERVICE_UUID);
  Serial.print("  特征UUID: ");
  Serial.println(CHARACTERISTIC_UUID);
  Serial.println("\n[提示] 请使用 Android 端扫描并连接此设备");
  Serial.println("[提示] 或使用 nRF Connect 等 BLE 工具测试\n");

  // LED 闪烁 3 次表示初始化完成
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // 最终启动完成提示
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  ✓✓✓ 初始化完成！程序开始运行 ✓✓✓  ║");
  Serial.println("║     等待蓝牙连接...                  ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

// ==================== 主循环 ====================

void loop() {
  #if DEBUG_ENABLE
  // 每 5 秒报告 I2C 请求统计(仅调试模式)
  if (millis() - last_request_report >= 5000) {
    last_request_report = millis();
    Serial.println("\n========== I2C 从机状态报告 ==========");
    Serial.print("Arduino 请求次数: ");
    Serial.println(i2c_request_count);
    Serial.print("Arduino 发送次数: ");
    Serial.println(i2c_receive_count);
    Serial.print("平均请求频率: ");
    if (millis() > 5000) {
      Serial.print(i2c_request_count * 1000.0 / millis());
      Serial.println(" 次/秒");
    } else {
      Serial.println("计算中...");
    }
    Serial.print("普通队列状态: ");
    Serial.print(cmdQueue.count);
    Serial.print("/");
    Serial.print(CMD_QUEUE_SIZE);
    Serial.println(" (当前/容量)");
    Serial.print("优先级队列状态: ");
    Serial.print(priorityQueue.count);
    Serial.print("/");
    Serial.print(PRIORITY_QUEUE_SIZE);
    Serial.println(" (当前/容量,停止指令专用)");
    Serial.print("总接收命令: ");
    Serial.println(total_commands_received);
    Serial.print("丢弃命令: ");
    Serial.print(commands_dropped);
    Serial.print(" (");
    if (total_commands_received > 0) {
      Serial.print((commands_dropped * 100.0) / total_commands_received, 1);
    } else {
      Serial.print("0.0");
    }
    Serial.println("%)");
    Serial.println("======================================\n");
  }

  // 每 2 分钟打印一次心跳(仅调试模式)
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 120000) {
    Serial.println("========================================");
    Serial.print("[心跳] 程序运行中... 运行时间: ");
    Serial.print(millis() / 1000);
    Serial.println(" 秒");
    Serial.print("[状态] 连接状态: ");
    Serial.println(deviceConnected ? "已连接" : "未连接");
    Serial.println("========================================");
    lastHeartbeat = millis();
  }
  #endif

  // 处理断开连接后的重新广播
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);  // 给蓝牙栈时间准备
    pServer->startAdvertising();  // 重新开始广播
    Serial.println("[BLE] 正在等待新的连接...");
    oldDeviceConnected = deviceConnected;
  }

  // 处理新连接（在 loop 中也打印连接信息）
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("\n!!! [LOOP] 检测到新连接 !!!\n");
    oldDeviceConnected = deviceConnected;
  }

  // 心跳指示（连接时 LED 慢闪）
  static unsigned long lastBlink = 0;
  if (deviceConnected) {
    if (millis() - lastBlink > 2000) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastBlink = millis();
    }
  }

  delay(10);
}
