/**
 * 双电机MCP关节零点标定与控制
 * 
 * 功能：
 * 1. 双电机（电机1、电机2）零点标定
 * 2. Fe（屈曲）和AA（外展/内收）耦合控制
 * 3. 零点数据EEPROM存储（掉电不丢失）
 * 4. 串口通信接口，支持上位机控制
 * 
 * 物理模型：
 * - 电机1位置 = Fe + AA
 * - 电机2位置 = Fe - AA
 * 
 * 硬件：Arduino + 2个带磁编码器的伺服/步进电机
 */

#include <EEPROM.h>

// ========== 配置参数 ==========
#define MOTOR_COUNT 2           // 电机数量
#define ENCODER_BITS 14         // 磁编码器位数（0-16383）
#define ENCODER_MAX 16384       // 编码器最大值

// EEPROM存储地址
#define EEPROM_MAGIC_ADDR 0     // 魔数地址（用于验证数据有效性）
#define EEPROM_ZERO1_ADDR 4    // 电机1零点地址
#define EEPROM_ZERO2_ADDR 8    // 电机2零点地址
#define EEPROM_MAGIC_VALUE 0x5A5A  // 魔数值，表示已写入有效数据

// 电机引脚定义（根据实际硬件修改）
#define MOTOR1_PIN_EN 5         // 电机1使能
#define MOTOR1_PIN_STEP 6       // 电机1步进
#define MOTOR1_PIN_DIR 7        // 电机1方向
#define MOTOR2_PIN_EN 8         // 电机2使能
#define MOTOR2_PIN_STEP 9       // 电机2步进
#define MOTOR2_PIN_DIR 10       // 电机2方向

// 编码器引脚定义（SPI接口）
#define ENCODER1_CS A0          // 编码器1片选
#define ENCODER2_CS A1          // 编码器2片选

// ========== 数据结构 ==========
struct MotorState {
  int32_t rawPosition;      // 原始编码器读数
  int32_t zeroOffset;       // 零点偏移（标定值）
  int32_t relativePos;      // 相对位置（raw - zero）
  bool isCalibrated;        // 是否已标定
};

struct MCPJoint {
  float Fe;                 // 屈曲角度（度）
  float AA;                 // 外展/内收角度（度）
  float motor1Target;       // 电机1目标位置
  float motor2Target;       // 电机2目标位置
};

// ========== 全局变量 ==========
MotorState motor1, motor2;
MCPJoint mcpJoint;
bool torqueEnabled = true;   // 力矩使能状态

// 串口命令缓冲区
String cmdBuffer = "";

// ========== 初始化函数 ==========
void setup() {
  Serial.begin(921600);      // 与上位机通信波特率
  while (!Serial) { ; }      // 等待串口连接（Leonardo/Micro需要）
  
  // 初始化引脚
  pinMode(MOTOR1_PIN_EN, OUTPUT);
  pinMode(MOTOR1_PIN_STEP, OUTPUT);
  pinMode(MOTOR1_PIN_DIR, OUTPUT);
  pinMode(MOTOR2_PIN_EN, OUTPUT);
  pinMode(MOTOR2_PIN_STEP, OUTPUT);
  pinMode(MOTOR2_PIN_DIR, OUTPUT);
  pinMode(ENCODER1_CS, OUTPUT);
  pinMode(ENCODER2_CS, OUTPUT);
  
  // 默认使能电机
  digitalWrite(MOTOR1_PIN_EN, LOW);
  digitalWrite(MOTOR2_PIN_EN, LOW);
  
  // 初始化编码器SPI
  SPI.begin();
  
  // 从EEPROM加载零点
  loadZeroFromEEPROM();
  
  Serial.println("[MCP双电机控制器] 启动成功");
  Serial.println("命令列表:");
  Serial.println("  ZERO1    - 标定电机1零点");
  Serial.println("  ZERO2    - 标定电机2零点");
  Serial.println("  ZERO_ALL - 同时标定双电机零点");
  Serial.println("  FE:xx    - 设置Fe角度（如 FE:30）");
  Serial.println("  AA:xx    - 设置AA角度（如 AA:15）");
  Serial.println("  POS      - 查询当前位置");
  Serial.println("  TORQUE:0 - 关闭力矩（自由旋转模式）");
  Serial.println("  TORQUE:1 - 使能力矩");
  Serial.println("  SAVE     - 保存零点到EEPROM");
  Serial.println("  STATUS   - 显示当前状态");
}

void loop() {
  // 读取编码器当前值
  readEncoders();
  
  // 处理串口命令
  processSerialCommands();
  
  // 更新电机位置（如果有目标位置）
  updateMotorPosition();
  
  delay(10);  // 100Hz更新率
}

// ========== 编码器读取函数 ==========
/**
 * 读取磁编码器值（SPI通信）
 * 适配14位磁编码器（如AS5048A、AS5047P等）
 */
int32_t readEncoder(uint8_t csPin) {
  uint16_t value = 0;
  
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  
  // SPI读取14位编码器值
  byte highByte = SPI.transfer(0x00);
  byte lowByte = SPI.transfer(0x00);
  
  digitalWrite(csPin, HIGH);
  
  // 组合14位数据（去掉校验位）
  value = ((highByte & 0x3F) << 8) | lowByte;
  
  return (int32_t)value;
}

void readEncoders() {
  motor1.rawPosition = readEncoder(ENCODER1_CS);
  motor2.rawPosition = readEncoder(ENCODER2_CS);
  
  // 计算相对位置（考虑环形编码器）
  if (motor1.isCalibrated) {
    motor1.relativePos = calculateRelativePos(motor1.rawPosition, motor1.zeroOffset);
  }
  if (motor2.isCalibrated) {
    motor2.relativePos = calculateRelativePos(motor2.rawPosition, motor2.zeroOffset);
  }
}

/**
 * 计算相对位置，正确处理编码器环形特性
 * 返回值范围: -8192 ~ +8191 (半圈)
 */
int32_t calculateRelativePos(int32_t raw, int32_t zero) {
  int32_t diff = raw - zero;
  
  // 处理环形编码器跨越0点的情况
  if (diff > ENCODER_MAX / 2) {
    diff -= ENCODER_MAX;
  } else if (diff < -ENCODER_MAX / 2) {
    diff += ENCODER_MAX;
  }
  
  return diff;
}

// ========== 零点标定函数 ==========
/**
 * 标定电机1零点
 * 记录当前编码器值为零点基准
 */
void calibrateMotor1() {
  readEncoders();
  motor1.zeroOffset = motor1.rawPosition;
  motor1.isCalibrated = true;
  
  Serial.print("[标定] 电机1零点已记录: ");
  Serial.println(motor1.zeroOffset);
  
  // 可选：立即保存到EEPROM
  // saveZeroToEEPROM();
}

/**
 * 标定电机2零点
 */
void calibrateMotor2() {
  readEncoders();
  motor2.zeroOffset = motor2.rawPosition;
  motor2.isCalibrated = true;
  
  Serial.print("[标定] 电机2零点已记录: ");
  Serial.println(motor2.zeroOffset);
}

/**
 * 同时标定两个电机零点
 */
void calibrateDualMotor() {
  readEncoders();
  motor1.zeroOffset = motor1.rawPosition;
  motor2.zeroOffset = motor2.rawPosition;
  motor1.isCalibrated = true;
  motor2.isCalibrated = true;
  
  Serial.println("[标定] 双电机零点已同时记录");
  Serial.print("  电机1零点: ");
  Serial.println(motor1.zeroOffset);
  Serial.print("  电机2零点: ");
  Serial.println(motor2.zeroOffset);
  Serial.println("[提示] 当前位置已设为零点，Fe=0, AA=0");
  
  // 自动保存到EEPROM
  saveZeroToEEPROM();
}

// ========== Fe/AA控制函数 ==========
/**
 * 设置MCP关节Fe（屈曲）和AA（外展/内收）角度
 * 根据耦合关系计算电机目标位置
 * 
 * 简化线性模型：
 *   电机1位置 = Fe + AA
 *   电机2位置 = Fe - AA
 */
void setMCPPosition(float fe, float aa) {
  // 限制角度范围
  fe = constrain(fe, -90.0, 90.0);   // Fe: -90° ~ +90°
  aa = constrain(aa, -45.0, 45.0);   // AA: -45° ~ +45°
  
  mcpJoint.Fe = fe;
  mcpJoint.AA = aa;
  
  // 耦合关系：计算电机目标位置
  // 实际应用中可能需要根据机械结构调整公式
  mcpJoint.motor1Target = fe + aa;
  mcpJoint.motor2Target = fe - aa;
  
  Serial.print("[MCP] Fe=");
  Serial.print(fe);
  Serial.print("°, AA=");
  Serial.print(aa);
  Serial.print("° → M1=");
  Serial.print(mcpJoint.motor1Target);
  Serial.print("°, M2=");
  Serial.print(mcpJoint.motor2Target);
  Serial.println("°");
}

// ========== 电机控制函数 ==========
void updateMotorPosition() {
  if (!motor1.isCalibrated || !motor2.isCalibrated) {
    return;  // 未标定不执行控制
  }
  
  // TODO: 实现PID控制，驱动电机到目标位置
  // 这里仅打印目标与实际位置的差值
  
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {  // 每秒打印一次
    lastPrintTime = millis();
    
    float m1Actual = motor1.relativePos * 360.0 / ENCODER_MAX;
    float m2Actual = motor2.relativePos * 360.0 / ENCODER_MAX;
    
    Serial.print("[位置] M1实际=");
    Serial.print(m1Actual);
    Serial.print("°(目标=");
    Serial.print(mcpJoint.motor1Target);
    Serial.print("°) M2实际=");
    Serial.print(m2Actual);
    Serial.print("°(目标=");
    Serial.print(mcpJoint.motor2Target);
    Serial.println("°)");
  }
}

/**
 * 设置力矩状态
 * torqueOn: true=使能力矩, false=无力矩/自由旋转
 */
void setTorque(bool torqueOn) {
  torqueEnabled = torqueOn;
  
  if (torqueOn) {
    digitalWrite(MOTOR1_PIN_EN, LOW);   // 使能
    digitalWrite(MOTOR2_PIN_EN, LOW);
    Serial.println("[力矩] 已使能");
  } else {
    digitalWrite(MOTOR1_PIN_EN, HIGH);  // 禁用（自由旋转）
    digitalWrite(MOTOR2_PIN_EN, HIGH);
    Serial.println("[力矩] 已关闭（自由旋转模式）");
  }
}

// ========== EEPROM存储函数 ==========
/**
 * 保存零点值到EEPROM（掉电不丢失）
 */
void saveZeroToEEPROM() {
  EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_VALUE);
  EEPROM.put(EEPROM_ZERO1_ADDR, motor1.zeroOffset);
  EEPROM.put(EEPROM_ZERO2_ADDR, motor2.zeroOffset);
  
  Serial.println("[存储] 零点值已保存到EEPROM（掉电不丢失）");
}

/**
 * 从EEPROM加载零点值
 */
void loadZeroFromEEPROM() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  
  if (magic == EEPROM_MAGIC_VALUE) {
    // 有效数据，读取零点
    EEPROM.get(EEPROM_ZERO1_ADDR, motor1.zeroOffset);
    EEPROM.get(EEPROM_ZERO2_ADDR, motor2.zeroOffset);
    motor1.isCalibrated = true;
    motor2.isCalibrated = true;
    
    Serial.println("[加载] 从EEPROM加载零点成功");
    Serial.print("  电机1零点: ");
    Serial.println(motor1.zeroOffset);
    Serial.print("  电机2零点: ");
    Serial.println(motor2.zeroOffset);
  } else {
    // 无有效数据，使用默认零点
    motor1.zeroOffset = 0;
    motor2.zeroOffset = 0;
    motor1.targetRelativePos = 0;
    motor2.targetRelativePos = 0;
    motor1.isCalibrated = false;
    motor2.isCalibrated = false;
    
    Serial.println("[加载] EEPROM无有效零点数据，请先进行标定");
  }
}

// ========== 串口命令处理 ==========
void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      // 命令结束，处理
      if (cmdBuffer.length() > 0) {
        executeCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }
}

void executeCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  Serial.print("[命令] ");
  Serial.println(cmd);
  
  if (cmd == "ZERO1") {
    calibrateMotor1();
    
  } else if (cmd == "ZERO2") {
    calibrateMotor2();
    
  } else if (cmd == "ZERO_ALL") {
    calibrateDualMotor();
    
  } else if (cmd.startsWith("MOVE1:")) {
    // 电机1相对移动（用于标定时的微调）
    int delta = cmd.substring(6).toInt();
    moveMotorRelative(1, delta);
    
  } else if (cmd.startsWith("MOVE2:")) {
    // 电机2相对移动（用于标定时的微调）
    int delta = cmd.substring(6).toInt();
    moveMotorRelative(2, delta);
    
  } else if (cmd.startsWith("FE:")) {
    float fe = cmd.substring(3).toFloat();
    setMCPPosition(fe, mcpJoint.AA);
    
  } else if (cmd.startsWith("AA:")) {
    float aa = cmd.substring(3).toFloat();
    setMCPPosition(mcpJoint.Fe, aa);
    
  } else if (cmd == "POS") {
    Serial.print("[位置] M1原始=");
    Serial.print(motor1.rawPosition);
    Serial.print(" 相对=");
    Serial.print(motor1.relativePos);
    Serial.print(" | M2原始=");
    Serial.print(motor2.rawPosition);
    Serial.print(" 相对=");
    Serial.println(motor2.relativePos);
    
  } else if (cmd == "TORQUE:0" || cmd == "TORQUE:OFF") {
    setTorque(false);
    
  } else if (cmd == "TORQUE:1" || cmd == "TORQUE:ON") {
    setTorque(true);
    
  } else if (cmd == "SAVE") {
    saveZeroToEEPROM();
    
  } else if (cmd == "STATUS") {
    printStatus();
    
  } else {
    Serial.println("[错误] 未知命令，输入 HELP 查看帮助");
  }
}

void printStatus() {
  Serial.println("\n========== 当前状态 ==========");
  Serial.print("电机1: 原始=");
  Serial.print(motor1.rawPosition);
  Serial.print(" 零点=");
  Serial.print(motor1.zeroOffset);
  Serial.print(" 相对=");
  Serial.print(motor1.relativePos);
  Serial.print(" 标定=");
  Serial.println(motor1.isCalibrated ? "是" : "否");
  
  Serial.print("电机2: 原始=");
  Serial.print(motor2.rawPosition);
  Serial.print(" 零点=");
  Serial.print(motor2.zeroOffset);
  Serial.print(" 相对=");
  Serial.print(motor2.relativePos);
  Serial.print(" 标定=");
  Serial.println(motor2.isCalibrated ? "是" : "否");
  
  Serial.print("\nMCP关节: Fe=");
  Serial.print(mcpJoint.Fe);
  Serial.print("°, AA=");
  Serial.print(mcpJoint.AA);
  Serial.println("°");
  
  Serial.print("力矩状态: ");
  Serial.println(torqueEnabled ? "使能" : "关闭");
  Serial.println("==============================\n");
}

// ========== 辅助函数 ==========
float constrain(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}
