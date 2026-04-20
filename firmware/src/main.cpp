#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <cstdarg>
#include <cstdio>

// ISR 上下文标志：当定时器回调在运行时置位，用于抑制 Serial 输出
volatile bool inIsrContext = false;

// HC-06 经典蓝牙接 Serial1（TTL 3.3V：TX1/RX1）；无 USB 时可仅靠蓝牙调试
HardwareSerial& BT_SERIAL = Serial1;
static constexpr uint32_t BT_HC06_BAUD = 115200;
bool useBluetoothTelemetry = false;  // false=USB发送完整数据，true=蓝牙发送精简数据

// 处理串口命令时指向发起命令的端口；为 nullptr 时 host* 同时发到 USB 与蓝牙（启动/急停等）
Print* cmdReplyPort = nullptr;

void hostPrintf(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (cmdReplyPort) {
    cmdReplyPort->print(buf);
    // 蓝牙口发来的命令，额外镜像到 USB，便于本地串口监视器观察
    if (cmdReplyPort == static_cast<Print*>(&BT_SERIAL)) {
      Serial.print(buf);
    }
  } else {
    Serial.print(buf);
  }
}

void hostPrintln(const char* s) {
  if (cmdReplyPort) {
    cmdReplyPort->println(s);
    // 蓝牙口发来的命令，额外镜像到 USB，便于本地串口监视器观察
    if (cmdReplyPort == static_cast<Print*>(&BT_SERIAL)) {
      Serial.println(s);
    }
  } else {
    Serial.println(s);
  }
}

// 周期性 JSON 遥测：根据开关选择 USB 或蓝牙通道
void telemetryPrintf(const char* fmt, ...) {
  char buf[640];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (useBluetoothTelemetry) {
    BT_SERIAL.print(buf);
  } else {
    Serial.print(buf);
  }
}

struct CmdReplyScope {
  Print* prev;
  explicit CmdReplyScope(Print* src) : prev(cmdReplyPort) { cmdReplyPort = src; }
  ~CmdReplyScope() { cmdReplyPort = prev; }
};

#include <IntervalTimer.h>
// 定时器前向声明与变量
void runControlLoopOnce();
IntervalTimer controlTimer;

// ============================================================================
// 全局安全状态（Global Safety State）
// ============================================================================
volatile bool isSystemError = false;     // 系统是否处于严重错误状态
volatile uint8_t errorMotorId = 0;       // 报错的电机ID
volatile uint8_t errorCode = 0;          // 错误码
bool errorPrinted = false;               // 错误信息是否已打印到串口

// ============================================================================
// 前向声明与结构体移动（解决编译依赖顺序问题）
// ============================================================================

// 传感器轮询状态
struct SensorPollingTimer {
  bool enabled;              // 是否启用轮询（在ctrlon开启时自动启用）
  uint32_t lastPollMs;      // 上次轮询的时间
  uint32_t pollIntervalMs;  // 轮询间隔（毫秒），建议 10-20ms
};
SensorPollingTimer sensorPolling = {false, 0, 20}; // 默认20ms间隔（50Hz）

// 步态相位枚举（提前定义，ControlLoop需要）
enum GaitPhase {
  PHASE_STANCE = 0,  // 支撑相
  PHASE_SWING = 1    // 摆动相
};

float getStancePct(GaitPhase phase, uint32_t nowMs);

// 控制循环状态结构体
struct ControlLoop {
  uint32_t lastControlMs;      // 上次控制循环时间（毫秒）
  uint32_t controlIntervalMs;  // 控制周期（毫秒），100Hz = 10ms
  bool controlEnabled;         // 是否启用控制循环
  uint32_t controlCount;       // 控制循环计数（用于调试）
  bool ankleTorqueReleased;    // 兼容旧逻辑（保留但不再频繁使用 STOP）
  bool anklePositionActive;    // 兼容 JSON 输出的 act 标志
  GaitPhase prevPhase;         // 用于检测相位边沿
  uint16_t motorSpeed;         // 踝关节电机速度参数（协议单位，电机轴速度 dps），可通过串口指令设置
};
ControlLoop controlLoop = {
  0,        // lastControlMs
  10,       // controlIntervalMs (100Hz = 10ms)
  false,    // controlEnabled
  0,        // controlCount
  false,    // ankleTorqueReleased
  false,    // anklePositionActive
  PHASE_STANCE,  // prevPhase
  5000      // motorSpeed (默认最大速度，协议单位，电机轴速度 dps)
};



// ============================================================================
// CAN 协议配置（根据《电机CAN总线通讯协议 V2.35》）
// ============================================================================

// 使用 Teensy 4.1 的 CAN1 控制器（对应底板 CAN 引脚）
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can1;

// 电机配置结构体
struct MotorConfig {
  uint8_t id;          // 协议中的电机 ID（1=髋，2=踝）
  float unitsPerDeg;    // 每 1° 对应的协议单位数
  int8_t dir;          // 方向系数（+1/-1），用于统一处理角度方向
  const char *name;     // 关节名称，便于调试打印
};

// 协议规定：位置和多圈角度的电机轴单位为 0.01°/LSB，即 1° = 100 单位（与具体电机 ID 无关）
// 现在我们希望对外全部使用「关节角度」：
//   - 髋关节：减速比为 1:36，则 1 关节度 = 36 电机轴度 => 1 关节度 = 36 * 100 = 3600 协议单位
//   - 踝关节：减速比为 1:10，则 1 关节度 = 10 电机轴度 => 1 关节度 = 10 * 100 = 1000 协议单位
// 因此：
//   - 髋关节 unitsPerDeg = 3600（1 关节度 = 3600 协议单位，对应电机轴减速比 36）
//   - 踝关节 unitsPerDeg = 1000（1 关节度 = 1000 协议单位，对应电机轴减速比 10）
// 方向系数 dir：
//   - 用于统一处理角度方向，在"角度↔协议单位"转换的唯一入口统一处理
//   - 逻辑角 * dir * unitsPerDeg -> 协议单位
//   - 协议单位 / unitsPerDeg * dir -> 逻辑角
MotorConfig hipMotor { 1, 3600.0f, +1, "Hip" };
MotorConfig ankleMotor { 2, 1000.0f, -1, "Ankle" };  // 默认+1，可根据实际电机方向调整

// ============================================================================
// 角度接口层：明确区分原始角和逻辑角
// ============================================================================
// 原始角（只在驱动层出现）：
//   - raw_units: 协议单位（int64_t，单位 0.01°/LSB）
//   - raw_deg_motor: 电机端角度（度，未考虑减速比）
// 逻辑角（上层唯一允许使用）：
//   - ankle_deg: 解剖角（0=90°中立位，背屈为正）
//   - hip_deg: 髋逻辑角（0=定义的参考姿态）

// 电机状态结构体（用于存储反馈数据）
struct MotorStatus {
  // ========== 原始角（驱动层） ==========
  int64_t raw_units;           // 原始协议单位（int64_t，单位 0.01°/LSB）
  float raw_deg_motor;         // 电机端角度（度，未考虑减速比和零点偏移）
  
  // ========== 逻辑角（上层接口） ==========
  float hip_deg;               // 髋逻辑角（度，0=参考姿态）- 仅用于髋关节
  float ankle_deg;             // 踝解剖角（度，0=90°中立位，背屈为正）- 仅用于踝关节
  
  // ========== 其他状态 ==========
  int16_t speed;               // 速度（dps）
  int16_t iq;                  // 电流（q轴电流，mA，用于阻力检测）
  int8_t temperature;          // 温度（℃）
  uint8_t motorState;          // 电机状态（0x00=开启，0x10=关闭）
  uint8_t errorState;          // 错误状态
  bool enabled;                // 是否使能
  uint32_t lastUpdateMs;       // 最后更新时间
  
  // ========== 兼容性字段（保留，但标记为废弃） ==========
  // 注意：这些字段保留用于向后兼容，但新代码应使用逻辑角接口
  int64_t multiTurnAngle;      // [废弃] 使用 raw_units 替代
  float angleDeg;              // [废弃] 使用 hip_deg 或 ankle_deg 替代
};

MotorStatus hipStatus = {0, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, false, 0, 0, 0.0f};
MotorStatus ankleStatus = {0, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, false, 0, 0, 0.0f};

// 踝关节零点偏移（用于标定）
// 在用户站立自然中立位时，读取的踝电机多圈编码器角度值
// 后续踝解剖角计算：ankle_deg = (pos_raw - ankle_zero_offset) * k_deg
int64_t ankle_zero_offset = 0;  // 协议单位（0.01°/LSB）
bool ankle_zero_calibrated = false;  // 是否已完成标定

// 髋关节参考姿态偏移（用于定义hip_deg=0的参考位置）
// 如果需要在特定姿态下定义hip_deg=0，可以设置此偏移
int64_t hip_reference_offset = 0;  // 协议单位（0.01°/LSB）
bool hip_reference_set = false;  // 是否已设置参考姿态

// ====== 髋关节力矩模式状态 ======
// 当为 true 时，主循环会定时向髋电机发送转矩闭环控制命令（CMD_TORQUE_CTRL）
bool hipTorqueMode = false;      // 髋力矩模式使能标志
int16_t hipIqTarget = 0;         // 目标转矩电流（iq），单位 mA（或驱动约定单位）
uint32_t hipTorqueLastSendMs = 0; // 上次发送时间

// ====== 踝关节力矩测试模式状态 ======
// 当为 true 时，主循环会定时向踝电机发送转矩闭环控制命令（CMD_TORQUE_CTRL）
bool ankleTorqueMode = false;        // 踝力矩模式使能标志（测试用）
int16_t ankleIqTargetManual = 0;     // 踝关节手动目标转矩电流
uint32_t ankleTorqueLastSendMs = 0;  // 上次发送时间

// ============================================================================
// 角度接口层：逻辑角获取函数（上层唯一允许使用的接口）
// ============================================================================

// 获取髋逻辑角（度，0=参考姿态）
float getHipDeg() {
  return hipStatus.hip_deg;
}

// 获取踝解剖角（度，0=90°中立位，背屈为正）
float getAnkleDeg() {
  return ankleStatus.ankle_deg;
}

// 获取髋原始协议单位（仅驱动层使用）
int64_t getHipRawUnits() {
  return hipStatus.raw_units;
}

// 获取踝原始协议单位（仅驱动层使用）
int64_t getAnkleRawUnits() {
  return ankleStatus.raw_units;
}

// 获取髋电机端角度（度，仅驱动层使用）
float getHipRawDegMotor() {
  return hipStatus.raw_deg_motor;
}

// 获取踝电机端角度（度，仅驱动层使用）
float getAnkleRawDegMotor() {
  return ankleStatus.raw_deg_motor;
}

// ============================================================================
// 髋关节信号预处理（用于步态相位识别）
// ============================================================================

// 髋关节信号预处理状态
struct HipSignalProcessor {
  float hip_f;          // 滤波后的髋角（度）
  float hip_f_prev;      // 上次滤波后的髋角（用于计算速度）
  float hip_vel;         // 髋角速度（度/秒）
  float hip_vel_f;       // 滤波后的髋角速度（度/秒）
  uint32_t lastUpdateMs; // 上次更新时间（毫秒）
  bool initialized;      // 是否已初始化
};

HipSignalProcessor hipProcessor = {0.0f, 0.0f, 0.0f, 0.0f, 0, false};

// 滤波参数（根据《调整开发计划V1.md》）
const float HIP_FILTER_ALPHA = 0.2f;   // 髋角滤波系数（α = 0.15~0.25，取0.2）
const float HIP_VEL_FILTER_BETA = 0.2f; // 速度滤波系数（β = 0.2）

// ============================================================================
// 步态检测算法参数（V2：基于零点的绝对+趋势判定）
// ============================================================================
// 核心改进：
// 1. 不再计算 hip_mean，直接使用 hip_f > 5.0f 判定 SWING 
//    只要腿向前迈出超过 5°且带有向前的速度，立刻判定为 SWING
//    这能显著提前踝关节背屈辅助的时机，有效防止拖地
//
// 2. 在 SWING 状态下，不再等待角度掉过中点，而是监测 hip_vel_f < -10.0f（速度变负）
//    这意味着腿已经摆到了最高点开始下落（Heel Strike 瞬间），此时切换到 STANCE 最符合生理步态
//
// 3. 增加幅值下限保护：effective_amp = max(detector.hip_amp, 15.0f)
//    当用户坐下或站立微动时，识别算法不会因为 hip_amp 变成 1-2 度而导致状态疯狂跳变
//
// 4. 调试建议：
//    - 观察串口绘图器：在 updateGaitPhaseDetector 中打印 hip_f 和 detector.state*10
//    - 调整 V_up：如果静止时容易误触发进入 SWING，调大 V_up（目前 10.0f）
//    - 如果起步辅助太迟，调小 V_up（目前 10.0f，之前是 20.0f）

// 更新髋关节信号预处理
// 输入：hip_raw（原始髋角，度）
// 输出：更新hipProcessor中的滤波值和速度
void updateHipSignalProcessor(float hip_raw) {
  uint32_t now = millis();
  
  // 初始化
  if (!hipProcessor.initialized) {
    hipProcessor.hip_f = hip_raw;
    hipProcessor.hip_f_prev = hip_raw;
    hipProcessor.hip_vel = 0.0f;
    hipProcessor.hip_vel_f = 0.0f;
    hipProcessor.lastUpdateMs = now;
    hipProcessor.initialized = true;
    return;
  }
  
  // 计算时间差（秒）
  float dt = (now - hipProcessor.lastUpdateMs) / 1000.0f;
  
  // 如果时间差太大（>500ms）或无效，说明数据不连续，重新初始化
  // 注意：放宽限制，因为CAN响应时间可能不稳定
  if (dt > 0.5f || dt <= 0.0f) {
    hipProcessor.hip_f = hip_raw;
    hipProcessor.hip_f_prev = hip_raw;
    hipProcessor.hip_vel = 0.0f;
    hipProcessor.hip_vel_f = 0.0f;
    hipProcessor.lastUpdateMs = now;
    return;
  }
  
  // 如果时间差太小（<1ms），跳过本次更新，避免除零或计算不稳定
  if (dt < 0.001f) {
    return;
  }
  
  // 1. 髋角低通滤波（EMA）
  // hip_f = hip_f + α * (hip_raw - hip_f)
  hipProcessor.hip_f = hipProcessor.hip_f + HIP_FILTER_ALPHA * (hip_raw - hipProcessor.hip_f);
  
  // 2. 髋角速度计算（差分）
  // hip_vel = (hip_f - hip_f_prev) / dt
  // 注意：使用滤波后的角度变化来计算速度
  float angle_diff = hipProcessor.hip_f - hipProcessor.hip_f_prev;
  hipProcessor.hip_vel = angle_diff / dt;
  
  // 3. 髋角速度再滤波（EMA）
  // hip_vel_f = hip_vel_f + β * (hip_vel - hip_vel_f)
  hipProcessor.hip_vel_f = hipProcessor.hip_vel_f + HIP_VEL_FILTER_BETA * (hipProcessor.hip_vel - hipProcessor.hip_vel_f);
  
  // 更新状态
  hipProcessor.hip_f_prev = hipProcessor.hip_f;
  hipProcessor.lastUpdateMs = now;
}

// ============================================================================
// 自适应阈值计算（用于步态相位识别）
// ============================================================================

// 滑动窗口大小：2秒 @ 100Hz = 200个数据点
#define HIP_WINDOW_SIZE 200

// 自适应阈值状态
struct AdaptiveThreshold {
  float window[HIP_WINDOW_SIZE];  // 滑动窗口（存储滤波后的髋角）
  uint16_t windowIndex;            // 当前窗口索引（循环缓冲区）
  uint16_t windowCount;            // 窗口中的数据数量（初始填充时使用）
  bool initialized;                // 是否已初始化
  
  float hip_mean;                  // 髋角均值（度）
  float hip_amp;                   // 髋角幅度（度）
  
  float A_up;                      // 角度阈值（向上）
  float A_dn;                      // 角度阈值（向下）
  float V_up;                      // 速度阈值（向上，度/秒）
  float V_dn;                      // 速度阈值（向下，度/秒）
  
  uint32_t lastUpdateMs;           // 上次更新时间
};

AdaptiveThreshold adaptiveThreshold = {
  {0}, 0, 0, false,
  0.0f, 0.0f,
  0.0f, 0.0f, 20.0f, -20.0f,
  0
};

// 防抖时间（毫秒）
const uint32_t T_HOLD_MS = 80;

// 步态检测参数调整指南
// ============================================================================
// 如果存在以下问题，按建议调整：
//
// 问题1：静止时容易误触发进入 SWING（状态疯狂闪烁）
//   -> 调大 V_up 值（当前 10.0f，可试试 15.0f）
//   -> V_up 是进入 SWING 时的速度阈值，值越大越难进入
//
// 问题2：起步辅助太迟，踝关节背屈反应慢
//   -> 调小 V_up 值（当前 10.0f，可试试 5.0f）
//   -> 降低 hip_f 阈值（当前 5.0f，可试试 3.0f）
//
// 问题3：在 SWING 相坚持太久，不能及时切换到 STANCE
//   -> 检查 hip_vel_f < -10.0f 的判定是否满足
//   -> 调大负速度阈值的绝对值（当前 -10.0f，可试试 -8.0f）
//
// 问题4：用户坐着或站立时微动导致状态跳变
//   -> 幅值下限已设置为 15.0f，一般不需要调整
// ============================================================================

// 更新自适应阈值
// 输入：hip_f（滤波后的髋角，度）
// 输出：更新adaptiveThreshold中的均值和阈值
void updateAdaptiveThreshold(float hip_f) {
  uint32_t now = millis();
  
  // 初始化
  if (!adaptiveThreshold.initialized) {
    // 填充窗口初始值
    for (uint16_t i = 0; i < HIP_WINDOW_SIZE; i++) {
      adaptiveThreshold.window[i] = hip_f;
    }
    adaptiveThreshold.windowIndex = 0;
    adaptiveThreshold.windowCount = HIP_WINDOW_SIZE;
    adaptiveThreshold.hip_mean = hip_f;
    adaptiveThreshold.hip_amp = 0.0f;
    adaptiveThreshold.A_up = 0.0f;
    adaptiveThreshold.A_dn = 0.0f;
    adaptiveThreshold.lastUpdateMs = now;
    adaptiveThreshold.initialized = true;
    return;
  }
  
  // 将新数据加入滑动窗口（覆盖最旧的数据）
  adaptiveThreshold.window[adaptiveThreshold.windowIndex] = hip_f;
  adaptiveThreshold.windowIndex = (adaptiveThreshold.windowIndex + 1) % HIP_WINDOW_SIZE;
  
  // 如果窗口还未填满，增加计数
  if (adaptiveThreshold.windowCount < HIP_WINDOW_SIZE) {
    adaptiveThreshold.windowCount++;
  }
  
  // 计算均值
  float sum = 0.0f;
  for (uint16_t i = 0; i < adaptiveThreshold.windowCount; i++) {
    sum += adaptiveThreshold.window[i];
  }
  adaptiveThreshold.hip_mean = sum / adaptiveThreshold.windowCount;
  
  // 计算幅度（max - min）
  float min_val = adaptiveThreshold.window[0];
  float max_val = adaptiveThreshold.window[0];
  for (uint16_t i = 1; i < adaptiveThreshold.windowCount; i++) {
    if (adaptiveThreshold.window[i] < min_val) {
      min_val = adaptiveThreshold.window[i];
    }
    if (adaptiveThreshold.window[i] > max_val) {
      max_val = adaptiveThreshold.window[i];
    }
  }
  adaptiveThreshold.hip_amp = max_val - min_val;
  
  // ========== 增加幅值下限保护 ==========
  // 使用有效幅度：max(detector.hip_amp, 15.0f)
  // 当用户坐下或站立微动时，识别算法不会因为 hip_amp 变成 1-2 度而导致状态疯狂跳变
  float effective_amp = (adaptiveThreshold.hip_amp < 15.0f) ? 15.0f : adaptiveThreshold.hip_amp;
  
  // 计算阈值（现在基于有效幅度，但实际新算法中不再使用A_up/A_dn）
  // 保留这些字段用于兼容性，但值为0（新算法使用绝对判定）
  adaptiveThreshold.A_up = 0.0f;
  adaptiveThreshold.A_dn = 0.0f;
  
  // V_up = +10 deg/s（判定进入SWING的速度阈值，从20改为10以更快响应）
  // 注意：负值用于STANCE判定（-10 deg/s表示速度过零）
  adaptiveThreshold.V_up = 10.0f;
  adaptiveThreshold.V_dn = -10.0f;
  
  adaptiveThreshold.lastUpdateMs = now;
}

// ============================================================================
// 步态相位识别（两态状态机）
// ============================================================================

// 步态相位枚举
// enum GaitPhase {
//   PHASE_STANCE = 0,  // 支撑相
//   PHASE_SWING = 1    // 摆动相
// };
// 已移动到文件头部


// 步态相位识别状态
struct GaitPhaseDetector {
  GaitPhase currentPhase;         // 当前相位
  uint32_t phaseStartMs;          // 当前相位开始时间（毫秒）
  uint32_t conditionHoldMs;       // 条件持续满足的时间（毫秒）
  bool initialized;               // 是否已初始化
  uint32_t lastUpdateMs;          // 上次更新时间
  float hip_max;                  // 当前SWING周期内探测到的最大髋关节角度（峰值）
  float hip_min;                  // 当前STANCE阶段内的髋关节最小角度（谷值）
  float hip_max_last;             // 上一SWING周期的最大髋角（用于进度映射幅值）
};

GaitPhaseDetector gaitPhaseDetector = {
  PHASE_STANCE, 0, 0, false, 0,
  0.0f, 0.0f, 20.0f   // 初始 hip_max_last 给一个合理幅值，避免初始化阶段分母过小
};

// 更新步态相位识别
// 输入：使用hipProcessor和adaptiveThreshold中的数据
// 输出：更新gaitPhaseDetector中的相位状态
void updateGaitPhaseDetector() {
  // 检查前置条件：信号处理和阈值计算必须已初始化
  if (!hipProcessor.initialized || !adaptiveThreshold.initialized) {
    return;
  }
  
  uint32_t now = millis();
  
  // 初始化：默认从支撑相开始
  if (!gaitPhaseDetector.initialized) {
    gaitPhaseDetector.currentPhase = PHASE_STANCE;
    gaitPhaseDetector.phaseStartMs = now;
    gaitPhaseDetector.conditionHoldMs = 0;
    gaitPhaseDetector.lastUpdateMs = now;
    // 初始化谷值与峰值
    float hip_f_init = hipProcessor.hip_f;
    gaitPhaseDetector.hip_min = hip_f_init;
    gaitPhaseDetector.hip_max = hip_f_init;
    gaitPhaseDetector.hip_max_last = hip_f_init + 15.0f;  // 给一个至少15°的初始幅值
    gaitPhaseDetector.initialized = true;
    return;
  }
  
  // 计算时间差
  uint32_t dt_ms = now - gaitPhaseDetector.lastUpdateMs;
  if (dt_ms == 0) {
    return;  // 避免除零
  }
  
  // 获取当前信号值
  float hip_f = hipProcessor.hip_f;
  float hip_vel_f = hipProcessor.hip_vel_f;
  float V_up = adaptiveThreshold.V_up;
  
  // 在SWING内实时更新本周期的最大髋角（峰值）
  if (gaitPhaseDetector.currentPhase == PHASE_SWING) {
    if (hip_f > gaitPhaseDetector.hip_max) {
      gaitPhaseDetector.hip_max = hip_f;
    }
  }

  // 在STANCE内实时更新本阶段谷值（最小髋角），用于相对增量判定
  if (gaitPhaseDetector.currentPhase == PHASE_STANCE) {
    if (hip_f < gaitPhaseDetector.hip_min) {
      gaitPhaseDetector.hip_min = hip_f;
    }
  }

  // ========== 基于相对增量 + 趋势判定 ==========
  // 进入SWING条件：从当前谷值抬升超过1.5°，且向前速度大于10°/s
  float delta_from_min = hip_f - gaitPhaseDetector.hip_min;
  bool swingConditionMet = (delta_from_min > 1.5f) && (hip_vel_f > 10.0f);
  
  // STANCE条件：在SWING状态下，监测速度过零（hip_vel_f < -10.0f）
  // 表示腿已经摆到最高点开始下落（Heel Strike瞬间），此时切换到STANCE最符合生理步态
  bool stanceConditionMet = (hip_vel_f < -10.0f);
  
  // 根据当前相位和条件，更新防抖计时器
  if (gaitPhaseDetector.currentPhase == PHASE_STANCE) {
    // 当前是支撑相，检查是否满足进入摆动相的条件
    if (swingConditionMet) {
      gaitPhaseDetector.conditionHoldMs += dt_ms;
    } else {
      // 条件不满足，重置计时器
      gaitPhaseDetector.conditionHoldMs = 0;
    }
    
    // 如果条件持续满足超过防抖时间，切换到摆动相
    if (gaitPhaseDetector.conditionHoldMs >= T_HOLD_MS) {
      gaitPhaseDetector.currentPhase = PHASE_SWING;
      gaitPhaseDetector.phaseStartMs = now;
      gaitPhaseDetector.conditionHoldMs = 0;  // 重置计时器
      // 进入SWING：以当前hf作为本周期的起始峰值
      gaitPhaseDetector.hip_max = hip_f;
    }
  } else {
    // 当前是摆动相，检查是否满足进入支撑相的条件
    if (stanceConditionMet) {
      gaitPhaseDetector.conditionHoldMs += dt_ms;
    } else {
      // 条件不满足，重置计时器
      gaitPhaseDetector.conditionHoldMs = 0;
    }
    
    // 如果条件持续满足超过防抖时间，切换到支撑相
    if (gaitPhaseDetector.conditionHoldMs >= T_HOLD_MS) {
      gaitPhaseDetector.currentPhase = PHASE_STANCE;
      gaitPhaseDetector.phaseStartMs = now;
      gaitPhaseDetector.conditionHoldMs = 0;  // 重置计时器
      // 退出SWING：记录本周期峰值供后续进度映射使用，并用当前hf初始化下一周期的谷值
      gaitPhaseDetector.hip_max_last = gaitPhaseDetector.hip_max;
      gaitPhaseDetector.hip_min = hip_f;
      gaitPhaseDetector.hip_max = hip_f;  // 为下一周期重新探测峰值做准备
    }
  }
  
  // ========== 调试输出 ==========
  // 打印 hip_f 和 detector.state*10 便于观察
  // 注意：每次更新时打印（可通过调整条件改为定期打印，避免过量）
  // static uint32_t lastDebugMs = 0;
  // if (now - lastDebugMs >= 100) {  // 每100ms打印一次，避免串口过载
  //   uint8_t stateValue = (gaitPhaseDetector.currentPhase == PHASE_SWING) ? 1 : 0;
  //   Serial.print("HipF:");
  //   Serial.print(hip_f, 2);
  //   Serial.print(" VelF:");
  //   Serial.print(hip_vel_f, 2);
  //   Serial.print(" State:");
  //   Serial.println(stateValue * 10);  // 状态放大便于观察（0=STANCE, 10=SWING）
  //   lastDebugMs = now;
  // }
  
  gaitPhaseDetector.lastUpdateMs = now;
}

// 获取当前步态相位（供其他模块调用）
GaitPhase getCurrentGaitPhase() {
  return gaitPhaseDetector.initialized ? gaitPhaseDetector.currentPhase : PHASE_STANCE;
}

// 获取当前相位持续时间（毫秒）
uint32_t getCurrentPhaseDurationMs() {
  if (!gaitPhaseDetector.initialized) {
    return 0;
  }
  return millis() - gaitPhaseDetector.phaseStartMs;
}

// ============================================================================
// 摆动相进度计算（用于踝控制）
// ============================================================================
// 
// 轨迹百分比映射（进阶功能）：
// 除了时间相位（s = t_swing / Ts），还可利用空间相位进度：
//   space_phase = (hip_f - 0) / hip_max
// 其中：
//   - hip_f：当前髋角（度）
//   - hip_max：摆动周期内髋角的最大值（自适应计算）
//
// 相比于基于时间的百分比，这种方式更稳定：
//   - 用户摆腿快 -> 辅助就快
//   - 用户摆腿慢 -> 辅助就慢
//   - 不受周期变化影响，更符合人体肌肉的反馈需求
//
// 目前代码采用时间相位（swing_progress = t_swing / Ts）
// 如需使用空间相位，可在 updateSwingProgress() 中补充计算逻辑
// ============================================================================

// 摆动相进度状态
struct SwingProgress {
  float Ts;                        // 摆动平均周期（秒）
  float t_swing;                   // 当前摆动时长（秒）
  float swing_progress;            // 摆动进度 s (0.0 ~ 1.0)
  bool initialized;                // 是否已初始化
  GaitPhase lastPhase;             // 上次相位（用于检测相位切换）
};

SwingProgress swingProgress = {
  0.4f,    // 初始 Ts = 0.4s
  0.0f,    // 初始 t_swing = 0
  0.0f,    // 初始进度 = 0
  false,   // 未初始化
  PHASE_STANCE
};

// 更新摆动相进度计算
// 输入：使用gaitPhaseDetector中的相位信息
// 输出：更新swingProgress中的进度值
void updateSwingProgress() {
  // 检查前置条件：相位识别必须已初始化
  if (!gaitPhaseDetector.initialized) {
    return;
  }
  
  // 初始化
  if (!swingProgress.initialized) {
    swingProgress.Ts = 0.4f;  // 初始 Ts = 0.4s
    swingProgress.t_swing = 0.0f;
    swingProgress.swing_progress = 0.0f;
    swingProgress.lastPhase = gaitPhaseDetector.currentPhase;
    swingProgress.initialized = true;
  }
  
  // 检测相位切换
  GaitPhase currentPhase = gaitPhaseDetector.currentPhase;
  bool phaseChanged = (currentPhase != swingProgress.lastPhase);
  
  // 计算当前摆动进度
  if (currentPhase == PHASE_SWING) {
    // 当前是摆动相，计算进度
    uint32_t swingDurationMs = getCurrentPhaseDurationMs();
    swingProgress.t_swing = swingDurationMs / 1000.0f;  // 转换为秒
    
    // 空间偏移映射进度：
    //   s = (hf - hip_min) / max(hip_max_last - hip_min, 15.0)
    // 这样0点永远对齐“起摆物理起点”（STANCE阶段的谷值），幅值使用上一周期的峰-谷差
    float hip_f = hipProcessor.hip_f;           // hf
    float hip_min = gaitPhaseDetector.hip_min;  // 当前STANCE阶段谷值
    float hip_max_last = gaitPhaseDetector.hip_max_last;  // 上一SWING周期峰值

    float amp = hip_max_last - hip_min;
    if (amp < 15.0f) amp = 15.0f;  // 幅值下限15度，避免分母过小

    float s = (hip_f - hip_min) / amp;
    swingProgress.swing_progress = constrain(s, 0.0f, 1.0f);
  } else {
    // 当前是支撑相，进度为0
    swingProgress.t_swing = 0.0f;
    swingProgress.swing_progress = 0.0f;
  }
  
  // 处理相位切换（在更新t_swing之后）
  if (phaseChanged) {
    // 相位切换了
    // 更新上次相位
    swingProgress.lastPhase = currentPhase;
  }
}

// 获取当前摆动进度（0.0 ~ 1.0）
float getSwingProgress() {
  return swingProgress.initialized ? swingProgress.swing_progress : 0.0f;
}

// 获取摆动平均周期（秒）
float getSwingAveragePeriod() {
  return swingProgress.initialized ? swingProgress.Ts : 0.4f;
}

// 获取当前摆动时长（秒）
float getCurrentSwingDuration() {
  return swingProgress.initialized ? swingProgress.t_swing : 0.0f;
}

// ============================================================================
// 4相步态识别（PhaseDetect4）
//
// 在现有2相（STANCE/SWING）基础上，对支撑相细分为3个子相，共4相：
//
//   P1 承重期 LOADING      — 踵触地→足掌着地（0~20%支撑进度）
//                            不施加主动助力；缓冲着地冲击；系统软启动
//   P2 支撑中期 MID_STANCE — 足掌着地→踵离地（20~65%支撑进度）
//                            踝中立位跟随；监测患者主动背屈意图；不干预
//   P3 推进期 PUSH_OFF     — 踵离地→趾离地（65~100%支撑进度）
//                            踝跖屈蹬地助力（PhaseProfile曲线输出）；改善推进
//   P4 摆动相 SWING        — 趾离地→踵触地
//                            踝背屈助力（PhaseProfile曲线软启动→保持）；防止拖地
//
// 切换条件：
//   SWING→P1: 底层2相检测到STANCE（hip_vel_f < -10 deg/s，T_hold防抖）
//   P1→P2:    stance_pct ≥ 20%（防抖40ms）或超时800ms强制推进
//   P2→P3:    stance_pct ≥ 65%（防抖40ms）；超时2.5s触发退化
//   P3→P4:    底层2相检测到SWING（delta_from_min > 1.5°且vel > 10 deg/s，T_hold防抖）
//   P4→P1:    底层2相检测到STANCE（新步开始）；超时1.5s触发退化
//
// 退化策略：
//   超时退化 — 任一相停留超过各自超时阈值（见 PHASE4_TIMEOUT_Px_MS），触发退化模式
//   抖动退化 — 3秒内相切换次数≥14次，判定系统不稳定，触发退化模式
//   退化模式 — 仅做2相→4相直接映射（STANCE→MID_STANCE/PUSH_OFF，SWING→SWING）
//   自动恢复 — 退化后6秒自动尝试恢复到完整4相检测
//   助力策略 — 退化为 true 时关闭 A1 跖屈与 A2 背屈 iq 助力及踝背屈参考策略（相位不可靠）
// ============================================================================

// 4相枚举
enum GaitPhase4 {
  PHASE4_LOADING    = 0,  // 承重期 (Loading Response)
  PHASE4_MID_STANCE = 1,  // 支撑中期 (Mid-Stance)
  PHASE4_PUSH_OFF   = 2,  // 推进期/蹬地 (Terminal Stance / Push-Off)
  PHASE4_SWING      = 3   // 摆动相 (Swing)
};

// 相位曲线参数结构体（每相3参数 + 斜率限制）
// 曲线形状：Gaussian窗函数
//   output = amp * exp(-0.5 * ((progress - center) / width)^2)
struct PhaseProfile {
  float amp;         // 峰值助力因子（0.0 ~ 1.0）
  float center;      // 曲线中心（相内归一化进度，0.0 ~ 1.0）
  float width;       // 曲线半宽 sigma（0.0 ~ 1.0，越大越平缓）
  float ramp_limit;  // 每10ms控制周期的最大输出变化量（软斜率限制，归一化）
};

// 4相默认曲线参数（P1/P2无助力，P3蹬地，P4背屈）
PhaseProfile phaseProfiles[4] = {
  // P1_LOADING: 无助力（amp=0），仅软启动缓冲
  {0.0f, 0.5f,  0.40f, 0.03f},
  // P2_MID_STANCE: 无主动助力（amp=0），监测患者主动意图
  {0.0f, 0.5f,  0.40f, 0.03f},
  // P3_PUSH_OFF: 蹬地助力，峰值在相内40%处（踵离地后快速上升）
  {1.0f, 0.40f, 0.32f, 0.08f},
  // P4_SWING: 背屈助力，峰值在相内30%处（趾离地后快速背屈保持）
  {1.0f, 0.30f, 0.38f, 0.08f},
};

// 计算相位曲线输出（Gaussian窗函数）
// 输入：profile（相位参数）、progress（相内进度 0~1）
// 输出：0.0 ~ profile.amp
float computePhaseProfileOutput(const PhaseProfile& profile, float progress) {
  if (progress < 0.0f) progress = 0.0f;
  if (progress > 1.0f) progress = 1.0f;
  float sigma = (profile.width > 0.01f) ? profile.width : 0.01f;
  float z = (progress - profile.center) / sigma;
  float gauss = expf(-0.5f * z * z);
  return profile.amp * gauss;
}

// ---- 4相切换阈值参数 ----
// 支撑相内子相切换点（归一化支撑进度，基于stanceProg自适应值）
const float    PHASE4_P1P2_STANCE_PCT = 0.20f;  // P1→P2切换（支撑进度20%，约120ms@0.6s周期）
const float    PHASE4_P2P3_STANCE_PCT = 0.65f;  // P2→P3切换（支撑进度65%，确保早于蹬地窗0.75）

// 子相内防抖时间（比主相位防抖T_HOLD_MS=80ms短，因支撑进度曲线更平滑）
const uint32_t PHASE4_DEBOUNCE_MS     = 40;

// 各相超时退化阈值
const uint32_t PHASE4_TIMEOUT_P1_MS  = 800;    // P1承重期：超时强制推进到P2
const uint32_t PHASE4_TIMEOUT_P2_MS  = 4000;   // P2中期：放宽超时阈值，避免正常慢步被误判退化
const uint32_t PHASE4_TIMEOUT_P3_MS  = 800;    // P3推进期：超时触发退化（蹬地期不应超过0.8s）
const uint32_t PHASE4_TIMEOUT_P4_MS  = 1500;   // P4摆动相：超时触发退化（摆动不应超过1.5s）

// 抖动退化检测参数
const uint32_t PHASE4_UNSTABLE_WINDOW_MS    = 3000; // 抖动检测时间窗口（3秒）
const uint8_t  PHASE4_UNSTABLE_TRANS_THRESH = 14;   // 3秒内子相切换≥14次则触发退化
const uint32_t PHASE4_RECOVERY_MS           = 2000; // 退化后2秒自动尝试恢复，避免长期锁定退化

// 4相状态机结构体
struct GaitPhase4Detector {
  GaitPhase4  currentPhase;         // 当前4相
  uint32_t    phaseStartMs;         // 当前相开始时间（毫秒）
  float       phaseProgress;        // 当前相内归一化进度（0.0 ~ 1.0）
  float       profileOutput;        // 相位曲线输出（Gaussian，0.0 ~ amp）

  // 子相切换防抖计时器
  uint32_t    conditionHoldP2Ms;    // P1→P2条件持续计时（毫秒）
  uint32_t    conditionHoldP3Ms;    // P2→P3条件持续计时（毫秒）

  // 退化检测
  bool        degraded;             // 是否处于退化模式（2相直接映射）
  uint32_t    degradedStartMs;      // 退化开始时间
  uint8_t     transitionCount;      // 最近窗口内子相切换计数
  uint32_t    transitionWindowMs;   // 切换计数窗口起始时间

  bool        initialized;
  uint32_t    lastUpdateMs;
};

GaitPhase4Detector phase4Det = {
  PHASE4_LOADING,  // currentPhase
  0,               // phaseStartMs
  0.0f,            // phaseProgress
  0.0f,            // profileOutput
  0, 0,            // conditionHoldP2Ms, conditionHoldP3Ms
  false, 0,        // degraded, degradedStartMs
  0, 0,            // transitionCount, transitionWindowMs
  false,           // initialized
  0                // lastUpdateMs
};

// 登记一次子相切换，维护抖动计数器；超过阈值触发退化
void phase4RegisterTransition(uint32_t nowMs) {
  // 窗口超时则重置计数
  if (nowMs - phase4Det.transitionWindowMs >= PHASE4_UNSTABLE_WINDOW_MS) {
    phase4Det.transitionCount = 0;
    phase4Det.transitionWindowMs = nowMs;
  }
  if (phase4Det.transitionCount < 255) {
    phase4Det.transitionCount++;
  }
  if (!phase4Det.degraded &&
      phase4Det.transitionCount >= PHASE4_UNSTABLE_TRANS_THRESH) {
    phase4Det.degraded = true;
    phase4Det.degradedStartMs = nowMs;
  }
}

// 切换到指定4相（重置进度与防抖计时器，登记切换）
void phase4SwitchTo(GaitPhase4 newPhase, uint32_t nowMs) {
  phase4Det.currentPhase      = newPhase;
  phase4Det.phaseStartMs      = nowMs;
  phase4Det.phaseProgress     = 0.0f;
  phase4Det.conditionHoldP2Ms = 0;
  phase4Det.conditionHoldP3Ms = 0;
  phase4RegisterTransition(nowMs);
}

// 更新4相步态状态机（100Hz，在runControlLoopOnce中调用）
// 依赖：gaitPhaseDetector、swingProgress、stanceProg 均已更新
void updateGaitPhase4Detector() {
  if (!gaitPhaseDetector.initialized || !swingProgress.initialized) return;

  uint32_t nowMs = millis();

  // --- 初始化 ---
  if (!phase4Det.initialized) {
    GaitPhase base = gaitPhaseDetector.currentPhase;
    phase4Det.currentPhase      = (base == PHASE_SWING) ? PHASE4_SWING : PHASE4_LOADING;
    phase4Det.phaseStartMs      = nowMs;
    phase4Det.phaseProgress     = 0.0f;
    phase4Det.profileOutput     = 0.0f;
    phase4Det.degraded          = false;
    phase4Det.transitionCount   = 0;
    phase4Det.transitionWindowMs = nowMs;
    phase4Det.initialized       = true;
    phase4Det.lastUpdateMs      = nowMs;
    return;
  }

  uint32_t dt_ms = nowMs - phase4Det.lastUpdateMs;
  if (dt_ms == 0) {
    return;  // 避免在同一ms内重复执行
  }
  phase4Det.lastUpdateMs = nowMs;

  // ---- 退化模式处理 ----
  if (phase4Det.degraded) {
    // 超过恢复等待时间，重置退化并回到4相检测
    if (nowMs - phase4Det.degradedStartMs >= PHASE4_RECOVERY_MS) {
      phase4Det.degraded = false;
      phase4Det.transitionCount = 0;
      phase4Det.transitionWindowMs = nowMs;
      GaitPhase base = gaitPhaseDetector.currentPhase;
      phase4Det.currentPhase  = (base == PHASE_SWING) ? PHASE4_SWING : PHASE4_LOADING;
      phase4Det.phaseStartMs  = nowMs;
      phase4Det.phaseProgress = 0.0f;
      // 继续执行主状态机（不return，phaseDur=0安全）
    } else {
      // 退化期间：2相直接映射到4相（不细分支撑子相）
      GaitPhase base = gaitPhaseDetector.currentPhase;
      if (base == PHASE_SWING) {
        if (phase4Det.currentPhase != PHASE4_SWING) {
          phase4Det.currentPhase = PHASE4_SWING;
          phase4Det.phaseStartMs = nowMs;
        }
        phase4Det.phaseProgress = getSwingProgress();
      } else {
        // STANCE → 仅按支撑进度切换MID_STANCE / PUSH_OFF，跳过LOADING细分
        float sp = getStancePct(PHASE_STANCE, nowMs);
        GaitPhase4 target = (sp >= PHASE4_P2P3_STANCE_PCT) ?
                            PHASE4_PUSH_OFF : PHASE4_MID_STANCE;
        if (phase4Det.currentPhase != target) {
          phase4Det.currentPhase = target;
          phase4Det.phaseStartMs = nowMs;
        }
        phase4Det.phaseProgress = sp;
      }
      phase4Det.profileOutput = computePhaseProfileOutput(
          phaseProfiles[(int)phase4Det.currentPhase], phase4Det.phaseProgress);
      return;
    }
  }

  // ---- 获取底层状态 ----
  GaitPhase basePhase  = gaitPhaseDetector.currentPhase;
  float     swing_pct  = getSwingProgress();
  float     stance_pct = getStancePct(basePhase, nowMs);
  uint32_t  phaseDur   = nowMs - phase4Det.phaseStartMs;

  // ========================================================================
  // 主状态机
  // ========================================================================
  switch (phase4Det.currentPhase) {

    // ---- P1: 承重期 ----
    // 入口条件：SWING→STANCE（底层heel strike检测）
    // 出口条件：支撑进度≥20%（足掌着地完成）或超时800ms强制推进
    // 动作意图：不施加主动助力；确保系统软启动；踝自然着地
    case PHASE4_LOADING: {
      // 异常快速切换：底层已进入SWING，直接跟随
      if (basePhase == PHASE_SWING) {
        phase4SwitchTo(PHASE4_SWING, nowMs);
        break;
      }
      // 超时强制推进（承重期卡死保护）
      if (phaseDur >= PHASE4_TIMEOUT_P1_MS) {
        phase4SwitchTo(PHASE4_MID_STANCE, nowMs);
        break;
      }
      // 正常切换：支撑进度达到P1P2边界 + 防抖
      if (stance_pct >= PHASE4_P1P2_STANCE_PCT) {
        phase4Det.conditionHoldP2Ms += dt_ms;
      } else {
        phase4Det.conditionHoldP2Ms = 0;
      }
      if (phase4Det.conditionHoldP2Ms >= PHASE4_DEBOUNCE_MS) {
        phase4SwitchTo(PHASE4_MID_STANCE, nowMs);
      } else {
        // P1内进度：以超时阈值归一化的时间进度
        phase4Det.phaseProgress = constrain(
            (float)phaseDur / (float)PHASE4_TIMEOUT_P1_MS, 0.0f, 1.0f);
      }
      break;
    }

    // ---- P2: 支撑中期 ----
    // 入口条件：P1结束（支撑进度≥20%）
    // 出口条件：支撑进度≥65%（踵即将离地）
    // 动作意图：踝中立位跟随；监测患者主动背屈意图；不主动干预
    case PHASE4_MID_STANCE: {
      // 底层提前进入SWING（极短步或检测提前），跳过推进期
      if (basePhase == PHASE_SWING) {
        phase4SwitchTo(PHASE4_SWING, nowMs);
        break;
      }
      // 超时退化（用户站立不动或传感器异常）
      if (phaseDur >= PHASE4_TIMEOUT_P2_MS) {
        phase4Det.degraded = true;
        phase4Det.degradedStartMs = nowMs;
        break;
      }
      // 正常切换：支撑进度达到P2P3边界 + 防抖
      if (stance_pct >= PHASE4_P2P3_STANCE_PCT) {
        phase4Det.conditionHoldP3Ms += dt_ms;
      } else {
        phase4Det.conditionHoldP3Ms = 0;
      }
      if (phase4Det.conditionHoldP3Ms >= PHASE4_DEBOUNCE_MS) {
        phase4SwitchTo(PHASE4_PUSH_OFF, nowMs);
      } else {
        // P2内进度：在[P1P2_PCT, P2P3_PCT]区间内归一化
        float range = PHASE4_P2P3_STANCE_PCT - PHASE4_P1P2_STANCE_PCT;
        if (range < 0.01f) range = 0.01f;
        phase4Det.phaseProgress = constrain(
            (stance_pct - PHASE4_P1P2_STANCE_PCT) / range, 0.0f, 1.0f);
      }
      break;
    }

    // ---- P3: 推进期/蹬地 ----
    // 入口条件：P2结束（支撑进度≥65%）
    // 出口条件：底层切换到SWING（趾离地检测）或超时800ms退化
    // 动作意图：踝跖屈蹬地助力（PhaseProfile Gaussian曲线）；改善推进期步态
    case PHASE4_PUSH_OFF: {
      // 正常出口：底层检测到趾离地进入SWING
      if (basePhase == PHASE_SWING) {
        phase4SwitchTo(PHASE4_SWING, nowMs);
        break;
      }
      // 超时退化（蹬地期不应持续超过0.8s）
      if (phaseDur >= PHASE4_TIMEOUT_P3_MS) {
        phase4Det.degraded = true;
        phase4Det.degradedStartMs = nowMs;
        break;
      }
      // P3内进度：在[P2P3_PCT, 1.0]区间内归一化
      float range = 1.0f - PHASE4_P2P3_STANCE_PCT;
      if (range < 0.01f) range = 0.01f;
      phase4Det.phaseProgress = constrain(
          (stance_pct - PHASE4_P2P3_STANCE_PCT) / range, 0.0f, 1.0f);
      break;
    }

    // ---- P4: 摆动相 ----
    // 入口条件：P3→SWING（趾离地，底层2相切换）
    // 出口条件：底层切换到STANCE（踵触地，新步开始）或超时1.5s退化
    // 动作意图：踝背屈助力（PhaseProfile Gaussian曲线）；防止脚尖拖地
    case PHASE4_SWING: {
      // 正常出口：底层检测到踵触地进入新步的承重期
      if (basePhase == PHASE_STANCE) {
        phase4SwitchTo(PHASE4_LOADING, nowMs);
        break;
      }
      // 超时退化（摆动相不应持续超过1.5s）
      if (phaseDur >= PHASE4_TIMEOUT_P4_MS) {
        phase4Det.degraded = true;
        phase4Det.degradedStartMs = nowMs;
        break;
      }
      // P4内进度：直接使用底层摆动进度（空间映射，与步速自适应）
      phase4Det.phaseProgress = swing_pct;
      break;
    }
  }

  // 更新相位曲线Gaussian输出（供PhaseProfileCurve模块使用）
  phase4Det.profileOutput = computePhaseProfileOutput(
      phaseProfiles[(int)phase4Det.currentPhase], phase4Det.phaseProgress);
}

// 获取当前4相枚举值（0=LOADING, 1=MID_STANCE, 2=PUSH_OFF, 3=SWING）
GaitPhase4 getCurrentGaitPhase4() {
  return phase4Det.initialized ? phase4Det.currentPhase : PHASE4_LOADING;
}

// 获取当前相内归一化进度（0.0 ~ 1.0）
float getPhase4Progress() {
  return phase4Det.initialized ? phase4Det.phaseProgress : 0.0f;
}

// 获取当前相位曲线Gaussian输出（0.0 ~ phaseProfiles[ph].amp）
float getPhase4ProfileOutput() {
  return phase4Det.initialized ? phase4Det.profileOutput : 0.0f;
}

// 是否处于退化模式（true=2相映射，false=完整4相）
bool isPhase4Degraded() {
  return phase4Det.initialized ? phase4Det.degraded : false;
}

// ============================================================================
// 踝背屈辅助策略（B人群核心）
// ============================================================================

// 辅助策略参数
#define ANKLE_THETA_LOW  2.0f   // 背屈窗口下限（度）
#define ANKLE_THETA_HIGH 6.0f   // 背屈窗口上限（度）
#define ANKLE_THETA_MIN  -15.0f // 安全限位下限（跖屈，度）
#define ANKLE_THETA_MAX  40.0f  // 安全限位上限（背屈，度）- swing相时背屈角度为40度

// 踝背屈辅助状态
struct AnkleAssistController {
  float theta_ref;              // 参考角度（度）
  float theta_target;           // S曲线目标角度（度）
  float assist_factor;          // 助力衰减因子（0.0 ~ 1.0）
  bool enabled;                 // 是否启用辅助
  bool initialized;             // 是否已初始化
  float dorsiflexion_target;    // 背屈目标角度（度），可通过串口指令设置
};

AnkleAssistController ankleAssist = {
  0.0f,           // theta_ref
  0.0f,           // theta_target
  1.0f,           // assist_factor
  false,          // enabled
  false,          // initialized
  ANKLE_THETA_HIGH // dorsiflexion_target (默认6.0度)
};

// 计算S曲线平滑函数
// 输入：s (0.0 ~ 1.0)
// 输出：u (0.0 ~ 1.0)，S曲线平滑值
float smoothStep(float s) {
  // u = s*s*(3 - 2*s)
  if (s <= 0.0f) {
    return 0.0f;
  } else if (s >= 1.0f) {
    return 1.0f;
  } else {
    return s * s * (3.0f - 2.0f * s);
  }
}

// 更新踝背屈辅助策略
// 输入：当前踝关节角度（度）、当前步态相位、摆动进度
// 输出：更新ankleAssist中的参考角度和助力因子
void updateAnkleAssistStrategy(float ankle_deg, GaitPhase currentPhase, float swing_progress) {
  // 初始化
  if (!ankleAssist.initialized) {
    ankleAssist.enabled = true;
    ankleAssist.initialized = true;
  }
  
  // 如果辅助未启用，直接返回
  if (!ankleAssist.enabled) {
    ankleAssist.theta_ref = ankle_deg;
    ankleAssist.theta_target = ankleAssist.dorsiflexion_target;
    ankleAssist.assist_factor = 0.0f;
    return;
  }
  
  // 产品策略：背屈参考仅在与 4 相一致的摆动相内更新；退化模式下关闭
  const bool df_phase_ok = (currentPhase == PHASE_SWING &&
                            getCurrentGaitPhase4() == PHASE4_SWING &&
                            !isPhase4Degraded());
  if (df_phase_ok) {
    // 立即触发目标角度：使用参数化的背屈目标角度，不再使用 S 曲线平滑
    ankleAssist.theta_ref = ankleAssist.dorsiflexion_target;
    ankleAssist.theta_target = ankleAssist.dorsiflexion_target;
    
    // assist_factor 软启动：在摆动相前 200ms 内，从 0 线性增加到 1.0
    // 这可以防止电流突然跳变，即使 target angle 已经到位
    uint32_t swingDuration = getCurrentPhaseDurationMs();
    if (swingDuration < 200) {
        ankleAssist.assist_factor = (float)swingDuration / 200.0f;
    } else {
        ankleAssist.assist_factor = 1.0f;  // 最大助力
    }
  } else {
    // 支撑相：跟随当前角度，不提供辅助
    ankleAssist.theta_ref = ankle_deg;
    ankleAssist.theta_target = ankleAssist.dorsiflexion_target;
    ankleAssist.assist_factor = 0.0f;
  }
  
  // 安全限位检查
  // 限制参考角度在安全范围内
  if (ankleAssist.theta_ref < ANKLE_THETA_MIN) {
    ankleAssist.theta_ref = ANKLE_THETA_MIN;
  } else if (ankleAssist.theta_ref > ANKLE_THETA_MAX) {
    ankleAssist.theta_ref = ANKLE_THETA_MAX;
  }
}

// 获取踝关节参考角度（度）
float getAnkleReferenceAngle() {
  return ankleAssist.initialized ? ankleAssist.theta_ref : 0.0f;
}

// 获取S曲线目标角度（度）
float getAnkleTargetAngle() {
  return ankleAssist.initialized ? ankleAssist.theta_target : ANKLE_THETA_LOW;
}

// 获取助力衰减因子（0.0 ~ 1.0）
float getAnkleAssistFactor() {
  return ankleAssist.initialized ? ankleAssist.assist_factor : 0.0f;
}

// 启用/禁用踝背屈辅助
void setAnkleAssistEnabled(bool enabled) {
  ankleAssist.enabled = enabled;
  if (!ankleAssist.initialized) {
    ankleAssist.initialized = true;
  }
}

// ============================================================================
// 顺从与软化控制（必须）
// ============================================================================

// 顺从控制参数（初始值，需通过日志校准）
#define COMPLIANCE_I1  500   // 轻度阻力阈值（mA，q轴电流）
#define COMPLIANCE_I2  1000  // 重度阻力阈值（mA，q轴电流）
#define COMPLIANCE_E1  3.0f  // 位置误差阈值1（度）
#define COMPLIANCE_E2  7.0f  // 位置误差阈值2（度）
#define COMPLIANCE_T_RESIST  250  // 阻力持续时间阈值（ms），用于退出COMPLIANT/HOLD状态

// 温度阈值（℃）
#define TEMP_MAX  70   // 最大允许温度
#define TEMP_WARN 60  // 温度警告阈值

// 通讯超时阈值（ms）
#define COMM_TIMEOUT_MS  500  // 通讯超时时间

// 控制状态枚举
enum ComplianceState {
  STATE_NORMAL = 0,      // 正常辅助
  STATE_COMPLIANT = 1,   // 降低maxSpeed，减缓推进
  STATE_HOLD = 2,        // 停止推进，保持当前位置
  STATE_FAULT_SAFE = 3   // 故障保护
};

// 顺从控制状态
struct ComplianceController {
  ComplianceState currentState;        // 当前状态
  ComplianceState lastState;           // 上次状态（用于状态切换检测）
  uint32_t stateStartMs;               // 当前状态开始时间（毫秒）
  uint32_t lowResistanceStartMs;       // 低阻力开始时间（用于退出条件）
  bool initialized;                    // 是否已初始化
  
  // 状态相关的控制参数
  float maxSpeedFactor;                // 速度因子（NORMAL=1.0, COMPLIANT=0.5, HOLD=0.0）
  float positionHold;                  // HOLD状态下的保持位置（度）
};

ComplianceController complianceCtrl = {
  STATE_NORMAL,    // currentState
  STATE_NORMAL,    // lastState
  0,               // stateStartMs
  0,               // lowResistanceStartMs
  false,           // initialized
  1.0f,            // maxSpeedFactor
  0.0f             // positionHold
};

// 更新顺从控制状态机
// 输入：当前踝关节角度、参考角度、电流、温度、通讯状态
// 输出：更新complianceCtrl中的状态和控制参数
void updateComplianceController(float ankle_deg, float theta_ref, int16_t iq_mA, int8_t temperature, bool commOk) {
  // 初始化
  if (!complianceCtrl.initialized) {
    complianceCtrl.currentState = STATE_NORMAL;
    complianceCtrl.lastState = STATE_NORMAL;
    complianceCtrl.stateStartMs = millis();
    complianceCtrl.lowResistanceStartMs = 0;
    complianceCtrl.maxSpeedFactor = 1.0f;
    complianceCtrl.positionHold = ankle_deg;
    complianceCtrl.initialized = true;
  }
  
  uint32_t now = millis();
  
  // 计算位置误差
  float positionError = fabsf(theta_ref - ankle_deg);
  
  // 计算电流绝对值（阻力）
  float iq_abs = fabsf((float)iq_mA);
  
  // 1. 故障检测（最高优先级）
  bool faultCondition = false;
  if (!commOk) {
    // 通讯超时
    faultCondition = true;
  } else if (temperature > TEMP_MAX) {
    // 超温
    faultCondition = true;
  } else if (ankle_deg < ANKLE_THETA_MIN || ankle_deg > ANKLE_THETA_MAX) {
    // 越界
    faultCondition = true;
  }
  
  if (faultCondition) {
    complianceCtrl.currentState = STATE_FAULT_SAFE;
    complianceCtrl.maxSpeedFactor = 0.0f;  // 停止运动
    complianceCtrl.positionHold = ankle_deg;  // 保持当前位置
    complianceCtrl.stateStartMs = now;
    return;
  }
  
  // 2. 状态转换逻辑（仅在非故障状态下）
  ComplianceState newState = complianceCtrl.currentState;
  
  // 检查是否满足进入COMPLIANT的条件
  bool compliantCondition = (iq_abs > COMPLIANCE_I1) || (positionError > COMPLIANCE_E1);
  
  // 检查是否满足进入HOLD的条件
  bool holdCondition = (iq_abs > COMPLIANCE_I2) || (positionError > COMPLIANCE_E2);
  
  // 检查是否满足退出条件（阻力低于I1）
  bool lowResistanceCondition = (iq_abs < COMPLIANCE_I1);
  
  switch (complianceCtrl.currentState) {
    case STATE_NORMAL:
      if (holdCondition) {
        // 直接进入HOLD状态
        newState = STATE_HOLD;
        complianceCtrl.positionHold = ankle_deg;  // 保持当前位置
      } else if (compliantCondition) {
        // 进入COMPLIANT状态
        newState = STATE_COMPLIANT;
      }
      break;
      
    case STATE_COMPLIANT:
      if (holdCondition) {
        // 升级到HOLD状态
        newState = STATE_HOLD;
        complianceCtrl.positionHold = ankle_deg;
      } else if (lowResistanceCondition) {
        // 开始计时低阻力时间
        if (complianceCtrl.lowResistanceStartMs == 0) {
          complianceCtrl.lowResistanceStartMs = now;
        } else {
          // 检查是否持续低阻力足够长时间
          uint32_t lowResistanceDuration = now - complianceCtrl.lowResistanceStartMs;
          if (lowResistanceDuration >= COMPLIANCE_T_RESIST) {
            // 退出到NORMAL状态
            newState = STATE_NORMAL;
            complianceCtrl.lowResistanceStartMs = 0;
          }
        }
      } else {
        // 阻力又升高了，重置低阻力计时
        complianceCtrl.lowResistanceStartMs = 0;
      }
      break;
      
    case STATE_HOLD:
      if (lowResistanceCondition) {
        // 开始计时低阻力时间
        if (complianceCtrl.lowResistanceStartMs == 0) {
          complianceCtrl.lowResistanceStartMs = now;
        } else {
          // 检查是否持续低阻力足够长时间
          uint32_t lowResistanceDuration = now - complianceCtrl.lowResistanceStartMs;
          if (lowResistanceDuration >= COMPLIANCE_T_RESIST) {
            // 先退回到COMPLIANT状态（而不是直接到NORMAL）
            newState = STATE_COMPLIANT;
            complianceCtrl.lowResistanceStartMs = 0;
          }
        }
      } else {
        // 阻力又升高了，重置低阻力计时
        complianceCtrl.lowResistanceStartMs = 0;
      }
      break;
      
    case STATE_FAULT_SAFE:
      // 故障状态需要手动恢复（通过命令或重启）
      // 这里可以添加自动恢复逻辑（如果故障条件消失）
      if (commOk && temperature <= TEMP_MAX && 
          ankle_deg >= ANKLE_THETA_MIN && ankle_deg <= ANKLE_THETA_MAX) {
        // 故障条件消失，可以恢复到NORMAL
        newState = STATE_NORMAL;
        complianceCtrl.lowResistanceStartMs = 0;
      }
      break;
  }
  
  // 状态切换处理
  if (newState != complianceCtrl.currentState) {
    complianceCtrl.lastState = complianceCtrl.currentState;
    complianceCtrl.currentState = newState;
    complianceCtrl.stateStartMs = now;
    
    // 根据新状态设置控制参数
    switch (newState) {
      case STATE_NORMAL:
        complianceCtrl.maxSpeedFactor = 1.0f;
        break;
      case STATE_COMPLIANT:
        complianceCtrl.maxSpeedFactor = 0.5f;  // 降低速度到50%
        break;
      case STATE_HOLD:
        complianceCtrl.maxSpeedFactor = 0.0f;  // 停止推进
        complianceCtrl.positionHold = ankle_deg;  // 保持当前位置
        break;
      case STATE_FAULT_SAFE:
        complianceCtrl.maxSpeedFactor = 0.0f;
        complianceCtrl.positionHold = ankle_deg;
        break;
    }
  }
  
  // 在HOLD状态下，更新参考角度为保持位置
  if (complianceCtrl.currentState == STATE_HOLD) {
    // theta_ref会被覆盖为positionHold（在主循环中处理）
  }
}

// 获取当前顺从控制状态
ComplianceState getComplianceState() {
  return complianceCtrl.initialized ? complianceCtrl.currentState : STATE_NORMAL;
}

// 获取速度因子（用于限制电机速度）
float getComplianceSpeedFactor() {
  return complianceCtrl.initialized ? complianceCtrl.maxSpeedFactor : 1.0f;
}

// 获取HOLD状态下的保持位置（度）
float getComplianceHoldPosition() {
  return complianceCtrl.initialized ? complianceCtrl.positionHold : 0.0f;
}

// 获取当前状态持续时间（毫秒）
uint32_t getComplianceStateDuration() {
  if (!complianceCtrl.initialized) {
    return 0;
  }
  return millis() - complianceCtrl.stateStartMs;
}

// 手动重置故障状态（用于调试和恢复）
void resetComplianceFault() {
  if (complianceCtrl.currentState == STATE_FAULT_SAFE) {
    complianceCtrl.currentState = STATE_NORMAL;
    complianceCtrl.lastState = STATE_FAULT_SAFE;
    complianceCtrl.stateStartMs = millis();
    complianceCtrl.maxSpeedFactor = 1.0f;
    complianceCtrl.lowResistanceStartMs = 0;
  }
}

// ============================================================================
// CAN ID 定义（根据协议文档）
// ============================================================================
#define CAN_CMD_BASE_ID       0x140  // 控制指令基地址（0x140 + ID）

// ============================================================================
// 命令字节定义（根据协议文档 V2.35）
// ============================================================================
#define CMD_MOTOR_CLOSE       0x80   // 电机关闭命令（掉电）
#define CMD_MOTOR_STOP        0x81   // 电机停止命令
#define CMD_MOTOR_RUN         0x88   // 电机运行命令（使能）
#define CMD_READ_STATUS1       0x9A   // 读取电机状态1和错误标志
#define CMD_CLEAR_ERROR        0x9B   // 清除电机错误标志
#define CMD_READ_STATUS2       0x9C   // 读取电机状态2（温度、电流、速度、编码器位置）
#define CMD_READ_STATUS3       0x9D   // 读取电机状态3（温度、3相电流）
#define CMD_READ_MULTI_ANGLE   0x92   // 读取多圈角度命令
#define CMD_POSITION_CTRL1     0xA3   // 多圈位置闭环控制命令1
#define CMD_POSITION_CTRL2     0xA4   // 多圈位置闭环控制命令2（带速度限制）
// 转矩闭环控制（协议 0xA1）
#define CMD_TORQUE_CTRL        0xA1   // 转矩闭环控制命令（iqControl）

// 板级自定义命令 ID（用于主控向本板发送控制指令）
#define BOARD_CMD_ID           0x200  // 自定义：主控->下位机命令ID
#define BOARD_CMD_ENABLE_HIP_TORQUE 0xD1 // DATA[0]=0xD1, DATA[1-2]=int16_t iq(mA)
#define BOARD_CMD_DISABLE_HIP_TORQUE 0xD0 // DATA[0]=0xD0

// ============================================================================
// 工具函数
// ============================================================================

// ============================================================================
// 角度接口层：逻辑角转换为原始协议单位（用于控制命令）
// ============================================================================
// 注意：控制命令输入统一使用逻辑角（hip_deg或ankle_deg）
// 此函数将逻辑角转换为协议单位，用于发送控制命令

// 逻辑角（deg）转换为协议单位（int32）
// 对于髋关节：逻辑角 -> 协议单位（考虑参考偏移和方向系数）
// 对于踝关节：逻辑角（解剖角） -> 协议单位（考虑零点偏移和方向系数）
// 方向系数统一处理：逻辑角 * dir * unitsPerDeg -> 协议单位
int32_t logicalAngleToUnits(const MotorConfig &m, float logicalDeg) {
  int64_t offset;
  if (m.id == 1) {
    // 髋关节：逻辑角 + 参考偏移 -> 协议单位
    offset = hip_reference_offset;
  } else if (m.id == 2) {
    // 踝关节：逻辑角（解剖角） + 零点偏移 -> 协议单位
    offset = ankle_zero_calibrated ? ankle_zero_offset : 0;
  } else {
    offset = 0;
  }
  
  // 逻辑角转换为协议单位：units = logicalDeg * dir * unitsPerDeg + offset
  // 方向系数 dir 统一处理角度方向
  int64_t units = static_cast<int64_t>(logicalDeg * m.dir * m.unitsPerDeg) + offset;
  
  // 限制在int32范围内
  if (units > INT32_MAX) units = INT32_MAX;
  if (units < INT32_MIN) units = INT32_MIN;
  
  return static_cast<int32_t>(units);
}

// [兼容性] 角度（deg）转换为协议单位（int32）- 保留用于向后兼容
// 注意：新代码应使用logicalAngleToUnits()，此函数假设输入为逻辑角
int32_t angleDegToUnits(const MotorConfig &m, float deg) {
  return logicalAngleToUnits(m, deg);
}

// 协议单位转换为角度（deg）
// 方向系数统一处理：协议单位 / unitsPerDeg * dir -> 逻辑角
float unitsToAngleDeg(const MotorConfig &m, int64_t units) {
  // 协议单位转换为逻辑角：logicalDeg = (units / unitsPerDeg) * dir
  // 方向系数 dir 统一处理角度方向
  return static_cast<float>(units) * m.dir / m.unitsPerDeg;
}

// 关节速度（deg/s）转换为电机轴速度（dps）
// 根据减速比：关节速度 * 减速比 = 电机轴速度
// 髋关节减速比 1:36，踝关节减速比 1:10
uint16_t jointSpeedToMotorSpeed(const MotorConfig &m, float jointSpeedDps) {
  // 减速比 = unitsPerDeg / 100（因为1关节度 = unitsPerDeg协议单位，而1协议单位 = 0.01电机轴度）
  // 所以减速比 = unitsPerDeg / 100
  float reductionRatio = m.unitsPerDeg / 100.0f;
  float motorSpeedDps = jointSpeedDps * reductionRatio;
  
  // 转换为uint16_t，并限制在合理范围内
  if (motorSpeedDps < 0.0f) motorSpeedDps = 0.0f;
  if (motorSpeedDps > 10000.0f) motorSpeedDps = 10000.0f;  // 上限保护
  
  return static_cast<uint16_t>(motorSpeedDps);
}

// ============================================================================
// CAN 通信函数（根据协议文档实现）
// ============================================================================

// 发送 CAN 命令帧（通用函数）
// 协议格式：DATA[0] = 命令字节，DATA[1-7] = 命令数据（小端序）
// printDebug: 是否打印TX调试信息（查询类命令通常设为false）
bool sendCanCommand(uint8_t motorId, uint8_t cmd, const uint8_t *data = nullptr, uint8_t dataLen = 0, bool printDebug = false) {
  CAN_message_t msg;
  msg.id = CAN_CMD_BASE_ID + motorId;  // 0x140 + 电机ID
  msg.len = 8;
  msg.flags.extended = 0;  // 标准帧
  
  // 清空缓冲区
  memset(msg.buf, 0, 8);
  
  // 字节0：命令字节
  msg.buf[0] = cmd;
  // CAN总线通信保护：同一个控制器ID发送间隔需大于0.25ms

  // 静态变量用于记录每个motorId上次发送时间
  static uint32_t lastSendUs[256] = {0}; // 用于微秒级别限制

  uint8_t id = CAN_CMD_BASE_ID + motorId; // 即msg.id，CAN总线控制器ID

  uint32_t nowUs = micros();
  uint32_t intervalUs = nowUs - lastSendUs[id];

  // 仅对同一控制器做保护（不同控制器不检查）
  if (lastSendUs[id] != 0 && intervalUs < 350) {
    // 若距离上次发给同一控制器不到0.25ms（250us），则等待直到足够
    delayMicroseconds(350 - intervalUs);
    nowUs = micros();
  }
  lastSendUs[id] = nowUs;
  
  // 字节1-7：命令数据（如果有）
  if (data != nullptr && dataLen > 0) {
    uint8_t copyLen = (dataLen > 7) ? 7 : dataLen;
    memcpy(&msg.buf[1], data, copyLen);
  }
  
  // 尝试发送 CAN 消息，如果发送队列满则返回 false
  if (can1.write(msg)) {
    if (printDebug && !inIsrContext) {
      Serial.printf("[TX] Motor %d, CMD=0x%02X, ID=0x%03X, Data: ", motorId, cmd, msg.id);
      for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", msg.buf[i]);
      }
      Serial.println();
    }
    return true;
  } else {
    // 发送失败（通常是发送队列满），记录错误但不阻塞
    static uint32_t lastErrorMs = 0;
    uint32_t now = millis();
    // 限制错误输出频率（每1秒最多输出一次），避免串口阻塞
    if (now - lastErrorMs >= 1000) {
      if (!inIsrContext) {
        Serial.printf("[TX ERROR] Motor %d, CMD=0x%02X failed (TX queue full?)\n", motorId, cmd);
      }
      lastErrorMs = now;
    }
    return false;
  }
}

// 发送转矩闭环控制命令（CMD_TORQUE_CTRL，协议 0xA1）
// 协议：DATA[0]=0xA1, DATA[4-5] = iqControl (int16_t, little-endian)
bool sendTorqueCommand(const MotorConfig &motor, int16_t iqControl) {
  uint8_t data[7];
  memset(data, 0, sizeof(data));
  // sendCanCommand 会将 data[0] 映射到 CAN DATA[1]。
  // 协议要求 0xA1 的 iqControl 位于 CAN DATA[4]/DATA[5]，
  // 因此这里应放在 data[3]/data[4]，避免整体左移 1 字节。
  data[3] = (uint8_t)(iqControl & 0xFF);         // -> CAN DATA[4]
  data[4] = (uint8_t)((iqControl >> 8) & 0xFF);  // -> CAN DATA[5]
  // 打开 printDebug 便于调试
  return sendCanCommand(motor.id, CMD_TORQUE_CTRL, data, 7, false);
}

// 使能电机（电机运行命令 0x88）
void enableMotor(const MotorConfig &motor) {
  if (sendCanCommand(motor.id, CMD_MOTOR_RUN)) {
    hostPrintf(">>> %s motor (ID=%d) ENABLED (CMD=0x88)\n", motor.name, motor.id);
    if (motor.id == 1) {
      hipStatus.enabled = true;
      hipStatus.motorState = 0x00;  // 开启状态
    } else if (motor.id == 2) {
      ankleStatus.enabled = true;
      ankleStatus.motorState = 0x00;  // 开启状态
    }
  }
}

// 掉电/失能电机（电机关闭命令 0x80）
void disableMotor(const MotorConfig &motor) {
  if (sendCanCommand(motor.id, CMD_MOTOR_CLOSE)) {
    hostPrintf(">>> %s motor (ID=%d) DISABLED (CMD=0x80)\n", motor.name, motor.id);
    if (motor.id == 1) {
      hipStatus.enabled = false;
      hipStatus.motorState = 0x10;  // 关闭状态
    } else if (motor.id == 2) {
      ankleStatus.enabled = false;
      ankleStatus.motorState = 0x10;  // 关闭状态
    }
  }
}

// 停止电机（电机停止命令 0x81）
void stopMotor(const MotorConfig &motor) {
  sendCanCommand(motor.id, CMD_MOTOR_STOP);
  hostPrintf(">>> %s motor (ID=%d) STOPPED (CMD=0x81)\n", motor.name, motor.id);
}

// 读取电机多圈角度（命令 0x92）
// 返回：true=发送成功，false=发送失败（CAN队列满）
bool requestMotorAngle(const MotorConfig &motor) {
  return sendCanCommand(motor.id, CMD_READ_MULTI_ANGLE, nullptr, 0, false);  // 查询命令不输出TX调试信息
}

// 清除电机错误标志（命令 0x9B）
void clearMotorError(const MotorConfig &motor) {
  sendCanCommand(motor.id, CMD_CLEAR_ERROR);
  hostPrintf(">>> %s motor (ID=%d) error cleared (CMD=0x9B)\n", motor.name, motor.id);
}

// 发送位置控制指令（多圈位置闭环控制命令1，0xA3）
// 协议格式：DATA[0]=0xA3, DATA[1-3]=NULL, DATA[4-7]=位置控制值（int32，小端序）
// 位置控制值单位：0.01°/LSB，即 36000 代表 360°
void sendPositionCommand(const MotorConfig &motor, float targetDeg) {
  // 输入：targetDeg为逻辑角（hip_deg或ankle_deg）
  // 将逻辑角转换为协议单位（考虑零点偏移）
  int32_t targetUnits = logicalAngleToUnits(motor, targetDeg);
  
  // 获取当前角度（用于调试）- 使用逻辑角接口
  float currentDeg;
  int64_t currentUnits;
  if (motor.id == 1) {
    currentDeg = getHipDeg();
    currentUnits = getHipRawUnits();
  } else {
    currentDeg = getAnkleDeg();
    currentUnits = getAnkleRawUnits();
  }
  float diffDeg = targetDeg - currentDeg;
  
  // 协议格式：DATA[0]=0xA3, DATA[1-3]=NULL(0x00), DATA[4-7]=位置控制值(int32，小端序)
  // 需要将位置控制值放在 DATA[4-7]，DATA[1-3] 保持为 0x00
  uint8_t data[7];
  data[0] = 0x00;  // DATA[1] = NULL
  data[1] = 0x00;  // DATA[2] = NULL
  data[2] = 0x00;  // DATA[3] = NULL
  // DATA[4-7] = 位置控制值（小端序）
  data[3] = targetUnits & 0xFF;           // DATA[4] = 低字节
  data[4] = (targetUnits >> 8) & 0xFF;    // DATA[5]
  data[5] = (targetUnits >> 16) & 0xFF;   // DATA[6]
  data[6] = (targetUnits >> 24) & 0xFF;  // DATA[7] = 高字节
  
  sendCanCommand(motor.id, CMD_POSITION_CTRL1, data, 7);
  hostPrintf(">>> %s: current=%.2f deg (%lld units), target=%.2f deg (%ld units), diff=%.2f deg (CMD=0xA3)\n", 
                motor.name, currentDeg, static_cast<long long>(currentUnits), 
                targetDeg, static_cast<long>(targetUnits), diffDeg);
}

// 发送位置控制指令（带速度限制，多圈位置闭环控制命令2，0xA4）
// 协议格式：DATA[0]=0xA4, DATA[1]=NULL, DATA[2-3]=速度限制（uint16，小端序），DATA[4-7]=位置控制值（int32，小端序）
// 返回：true=发送成功，false=发送失败（CAN队列满）
bool sendPositionCommandWithSpeed(const MotorConfig &motor, float targetDeg, uint16_t maxSpeed) {
  // 输入：targetDeg为逻辑角（hip_deg或ankle_deg）
  // 将逻辑角转换为协议单位（考虑零点偏移）
  int32_t targetUnits = logicalAngleToUnits(motor, targetDeg);
  
  // 协议格式：DATA[0]=0xA4, DATA[1]=NULL(0x00), DATA[2-3]=速度限制(uint16，小端序), DATA[4-7]=位置控制值(int32，小端序)
  uint8_t data[7];
  data[0] = 0x00;  // DATA[1] = NULL
  // DATA[2-3] = 速度限制（uint16，小端序）
  data[1] = maxSpeed & 0xFF;           // DATA[2] = 速度限制低字节
  data[2] = (maxSpeed >> 8) & 0xFF;    // DATA[3] = 速度限制高字节
  // DATA[4-7] = 位置控制值（int32，小端序）
  data[3] = targetUnits & 0xFF;           // DATA[4] = 位置控制值低字节
  data[4] = (targetUnits >> 8) & 0xFF;    // DATA[5]
  data[5] = (targetUnits >> 16) & 0xFF;   // DATA[6]
  data[6] = (targetUnits >> 24) & 0xFF;  // DATA[7] = 位置控制值高字节

  // INSERT_YOUR_CODE
  // 以1Hz频率打印A4指令和目标角度、速度
  {
    static uint32_t lastA4PrintMs = 0;
    uint32_t now = millis();
    if (now - lastA4PrintMs >= 1000) {
      lastA4PrintMs = now;
      hostPrintf("[A4 CMD] %s: target=%.2f deg, speed=%u dps\n", 
                    motor.name, targetDeg, maxSpeed);
    }
  }
  
  return sendCanCommand(motor.id, CMD_POSITION_CTRL2, data, 7);
  
}

// ============================================================================
// CAN 反馈帧处理（根据协议文档实现）
// ============================================================================

// 处理接收到的 CAN 反馈帧
// 反馈帧使用相同的 CAN ID（0x140 + ID）
void handleCanMessage(const CAN_message_t &msg) {
  // 判断是否为控制指令的回复帧（ID = 0x140 + 电机ID）
  if (msg.id >= CAN_CMD_BASE_ID && msg.id < CAN_CMD_BASE_ID + 33) {
    uint8_t motorId = msg.id - CAN_CMD_BASE_ID;
    MotorStatus *status = nullptr;
    const MotorConfig *motor = nullptr;
    
    if (motorId == 1) {
      status = &hipStatus;
      motor = &hipMotor;
    } else if (motorId == 2) {
      status = &ankleStatus;
      motor = &ankleMotor;
    }
    
    if (status != nullptr && motor != nullptr) {
      uint8_t cmd = msg.buf[0];
      
      // 根据命令字节解析不同的回复帧
      if (cmd == CMD_READ_MULTI_ANGLE) {
        // 读取多圈角度回复（0x92）
        // 多圈角度为 int64_t，单位 0.01°/LSB，小端序
        // DATA[1-7] = 多圈角度（7字节，int64的低7字节）
        // 注意：int64_t 是 8 字节，但回复只有 7 字节数据，需要符号扩展最高字节
        int64_t angle = 0;
        angle |= ((int64_t)msg.buf[1]);
        angle |= ((int64_t)msg.buf[2]) << 8;
        angle |= ((int64_t)msg.buf[3]) << 16;
        angle |= ((int64_t)msg.buf[4]) << 24;
        angle |= ((int64_t)msg.buf[5]) << 32;
        angle |= ((int64_t)msg.buf[6]) << 40;
        // 符号扩展：如果第7字节（msg.buf[6]）的最高位是1，说明是负数
        if (msg.buf[6] & 0x80) {
          // 负数，符号扩展到最高字节
          angle |= ((int64_t)0xFF) << 48;
          angle |= ((int64_t)0xFF) << 56;
        }
        
        // ========== 角度接口层：更新原始角和逻辑角 ==========
        // 1. 更新原始角（驱动层）
        status->raw_units = angle;
        status->raw_deg_motor = static_cast<float>(angle) / 100.0f;  // 协议单位转电机端角度（0.01°/LSB）
        
        // 2. 更新逻辑角（上层接口）
        if (motor->id == 1) {
          // 髋关节：计算髋逻辑角
          // hip_deg = (raw_units - hip_reference_offset) / unitsPerDeg
          int64_t offsetAngle = angle - hip_reference_offset;
          status->hip_deg = unitsToAngleDeg(*motor, offsetAngle);
          status->ankle_deg = 0.0f;  // 髋关节不使用此字段
        } else if (motor->id == 2) {
          // 踝关节：计算踝解剖角
          if (ankle_zero_calibrated) {
            // 已标定：ankle_deg = (raw_units - ankle_zero_offset) / unitsPerDeg
            // 0 = 90°中立位，背屈为正
            int64_t offsetAngle = angle - ankle_zero_offset;
            status->ankle_deg = unitsToAngleDeg(*motor, offsetAngle);
          } else {
            // 未标定：使用原始角度（但这不是真正的解剖角）
            status->ankle_deg = unitsToAngleDeg(*motor, angle);
          }
          status->hip_deg = 0.0f;  // 踝关节不使用此字段
        }
        
        // ========== 兼容性字段（向后兼容） ==========
        status->multiTurnAngle = status->raw_units;
        if (motor->id == 1) {
          status->angleDeg = status->hip_deg;
        } else {
          status->angleDeg = status->ankle_deg;
        }
        
        status->lastUpdateMs = millis();
        
        // 对于髋关节，更新信号预处理、自适应阈值、步态相位识别和摆动进度
        if (motor->id == 1) {
          updateHipSignalProcessor(status->hip_deg);
          // 使用滤波后的髋角更新自适应阈值
          if (hipProcessor.initialized) {
            updateAdaptiveThreshold(hipProcessor.hip_f);
            // 更新步态相位识别
            updateGaitPhaseDetector();
            // 更新摆动相进度计算
            updateSwingProgress();
            // 更新踝背屈辅助策略（需要髋关节相位和进度信息）
            if (ankleStatus.lastUpdateMs > 0 && 
                (millis() - ankleStatus.lastUpdateMs) < 200) {  // 确保踝关节数据是新鲜的
              GaitPhase currentPhase = getCurrentGaitPhase();
              float swing_progress = getSwingProgress();
              updateAnkleAssistStrategy(getAnkleDeg(), currentPhase, swing_progress);
              
              // // 更新顺从控制状态机（需要参考角度、电流、温度、通讯状态）
              // float theta_ref = getAnkleReferenceAngle();
              // bool commOk = (millis() - ankleStatus.lastUpdateMs) < COMM_TIMEOUT_MS;
              // updateComplianceController(getAnkleDeg(), theta_ref, ankleStatus.iq, 
              //                            ankleStatus.temperature, commOk);
            }
          }
        }
        
        // 简化输出：只显示角度数据
        // if (motor->id == 1) {
        //   Serial.printf("Hip: %.2f deg\n", status->angleDeg);
        // } else {
        //   if (ankle_zero_calibrated) {
        //     Serial.printf("Ankle: %.2f deg (calibrated, offset=%lld)\n", 
        //                  status->angleDeg, static_cast<long long>(ankle_zero_offset));
        //   } else {
        //     Serial.printf("Ankle: %.2f deg (raw, NOT calibrated!)\n", status->angleDeg);
        //   }
        // }
      }
      else if (cmd == CMD_READ_STATUS1) {
        // 读取电机状态1回复（0x9A）
        int8_t temperature = (int8_t)msg.buf[1];
        uint16_t voltage = msg.buf[2] | (msg.buf[3] << 8);
        uint16_t current = msg.buf[4] | (msg.buf[5] << 8);
        uint8_t motorState = msg.buf[6];
        uint8_t errorState = msg.buf[7];
        
        status->temperature = temperature;
        status->motorState = motorState;
        status->errorState = errorState;
        status->enabled = (motorState == 0x00);
        
        // if (!inIsrContext) {
        //   Serial.printf("[RX] %s: temp=%d℃, voltage=%.2fV, current=%.2fA, state=0x%02X, error=0x%02X, ID=0x%03X\n",
        //                 motor->name, temperature, voltage * 0.01f, current * 0.01f, 
        //                 motorState, errorState, msg.id);
        // }

        // ====================================================================
        // 严重错误检测（Emergency Stop）
        // ====================================================================
        // 如果电机报告错误（errorState != 0），立即停止所有控制!
        if (errorState != 0) {
          if (!isSystemError) {
            isSystemError = true;
            errorMotorId = motor->id;
            errorCode = errorState;
            
            // 紧急停止控制循环
            controlLoop.controlEnabled = false;
            // 紧急停止传感器轮询
            sensorPolling.enabled = false;
            
            // 下发停止命令（尝试停止电机）
            // 注意：在ISR中发送CAN可能会有风险，但为了安全必须尝试
            // sendCanCommand(motor->id, CMD_MOTOR_STOP); 
            // 更好的做法是依赖全局标志让主循环停止，或者电机内部保护
          }
        }
      }
      //else if (cmd == CMD_READ_STATUS2 || cmd == CMD_POSITION_CTRL1 || cmd == CMD_POSITION_CTRL2) {
      else if (cmd == CMD_READ_STATUS2) {
        // 读取电机状态2回复（0x9C）或位置控制回复（0xA3/0xA4）
        // 格式相同：温度、电流/功率、速度、编码器位置
        int8_t temperature = (int8_t)msg.buf[1];
        int16_t iq = msg.buf[2] | (msg.buf[3] << 8);
        int16_t speed = msg.buf[4] | (msg.buf[5] << 8);
        uint16_t encoder = msg.buf[6] | (msg.buf[7] << 8);
        
        status->temperature = temperature;
        status->speed = speed;
        status->iq = iq;  // 保存q轴电流（mA）
        status->lastUpdateMs = millis();
        
        // Serial.printf("[RX] %s: temp=%d℃, iq=%d, speed=%d dps, encoder=%u, ID=0x%03X, CMD=0x%02X\n",
        //               motor->name, temperature, iq, speed, encoder, msg.id, cmd);
      }
      // else {
      //   // 其他回复帧
      //   Serial.printf("[RX] %s: ID=0x%03X, CMD=0x%02X, Data: ",
      //                 motor->name, msg.id, cmd);
      //   for (int i = 0; i < msg.len; i++) {
      //     Serial.printf("%02X ", msg.buf[i]);
      //   }
      //   Serial.println();
      // }
    }
  } else {
    // 板级自定义命令（来自主控的控制指令），ID=BOARD_CMD_ID
    if (msg.id == BOARD_CMD_ID && msg.len >= 1) {
      uint8_t bcmd = msg.buf[0];
      if (bcmd == BOARD_CMD_ENABLE_HIP_TORQUE && msg.len >= 3) {
        int16_t iq = (int16_t)(msg.buf[1] | (msg.buf[2] << 8));
        hipTorqueMode = true;
        hipIqTarget = iq;
        if (!inIsrContext) {
          telemetryPrintf(">>> BOARD: Enable hip torque mode, iq=%d\n", hipIqTarget);
        }
        // 回复ACK
        CAN_message_t ack;
        ack.id = BOARD_CMD_ID;
        ack.len = 8;
        ack.flags.extended = 0;
        memset(ack.buf, 0, 8);
        ack.buf[0] = BOARD_CMD_ENABLE_HIP_TORQUE;
        ack.buf[1] = (uint8_t)(hipIqTarget & 0xFF);
        ack.buf[2] = (uint8_t)((hipIqTarget >> 8) & 0xFF);
        can1.write(ack);
        return;
      } else if (bcmd == BOARD_CMD_DISABLE_HIP_TORQUE) {
        hipTorqueMode = false;
        if (!inIsrContext) {
          telemetryPrintf(">>> BOARD: Disable hip torque mode\n");
        }
        CAN_message_t ack;
        ack.id = BOARD_CMD_ID;
        ack.len = 8;
        ack.flags.extended = 0;
        memset(ack.buf, 0, 8);
        ack.buf[0] = BOARD_CMD_DISABLE_HIP_TORQUE;
        can1.write(ack);
        return;
      }
    }

    // 其他未知帧，原样打印
    if (!inIsrContext) {
      telemetryPrintf("[RX] Unknown ID=0x%03X, DLC=%u, Data: ", msg.id, msg.len);
      for (int i = 0; i < msg.len; i++) {
        telemetryPrintf("%02X ", msg.buf[i]);
      }
      telemetryPrintf("\n");
    }
  }
}

// ============================================================================
// 摆动功能：基于当前角度，左右摆动给定角度
// ============================================================================

struct SwingState {
  bool active;
  float centerAngle;    // 中心角度（当前角度）
  float amplitude;      // 摆动幅度（度）
  float currentAngle;   // 当前目标角度
  bool direction;       // true=向右，false=向左
  uint32_t lastStepMs;
  uint32_t stepIntervalMs;  // 每步间隔（毫秒）
  const MotorConfig *motor;
};

SwingState hipSwing = {false, 0.0f, 0.0f, 0.0f, true, 0, 50, &hipMotor};  // 50ms间隔，更平滑
SwingState ankleSwing = {false, 0.0f, 0.0f, 0.0f, true, 0, 50, &ankleMotor};  // 50ms间隔，更平滑

void startSwing(SwingState &swing, const MotorConfig &motor, float amplitudeDeg) {
  MotorStatus *status = (motor.id == 1) ? &hipStatus : &ankleStatus;
  // 查询关节电机角度为最新状态
  requestMotorAngle(motor);
  // INSERT_YOUR_CODE
  delay(1000);
  {
    CAN_message_t inMsg;
    while (can1.read(inMsg)) {
      handleCanMessage(inMsg);
    }
  }

  
  swing.motor = &motor;
  // 以当前逻辑角为中心
  if (motor.id == 1) {
    swing.centerAngle = getHipDeg();
  } else {
    swing.centerAngle = getAnkleDeg();
  }
  swing.amplitude = amplitudeDeg;
  swing.currentAngle = swing.centerAngle;
  swing.direction = true;  // 先向右
  swing.lastStepMs = millis();
  swing.active = true;
  
  hostPrintf(">>> %s swing started: center=%.2f deg, amplitude=%.2f deg\n",
                motor.name, swing.centerAngle, swing.amplitude);
}

void stopSwing(SwingState &swing) {
  swing.active = false;
  hostPrintf(">>> %s swing stopped\n", swing.motor->name);
}

void updateSwing(SwingState &swing) {
  if (!swing.active || swing.motor == nullptr) return;
  
  uint32_t now = millis();
  if (now - swing.lastStepMs < swing.stepIntervalMs) return;
  
  swing.lastStepMs = now;
  
  // 如果幅度为0，不执行摆动
  if (swing.amplitude <= 0.01f) {
    swing.active = false;
    hostPrintf(">>> %s swing stopped: amplitude is zero\n", swing.motor->name);
    return;
  }
  
  // 计算下一步角度（使用更小的步进，使移动更平滑）
  float step = 0.2f;  // 每步0.2度，更平滑
  float previousAngle = swing.currentAngle;
  
  if (swing.direction) {
    swing.currentAngle += step;
    if (swing.currentAngle >= swing.centerAngle + swing.amplitude) {
      swing.currentAngle = swing.centerAngle + swing.amplitude;
      swing.direction = false;  // 转向
    }
  } else {
    swing.currentAngle -= step;
    if (swing.currentAngle <= swing.centerAngle - swing.amplitude) {
      swing.currentAngle = swing.centerAngle - swing.amplitude;
      swing.direction = true;  // 转向
    }
  }
  
  // 只有当角度发生变化时才发送位置指令（避免重复发送相同角度）
  // 使用带速度限制的位置控制，速度限制为30 dps，使移动更平滑
  if (abs(swing.currentAngle - previousAngle) > 0.01f) {
    sendPositionCommandWithSpeed(*swing.motor, swing.currentAngle, 200);  // 30 dps 速度限制
  }
}

// ============================================================================
// 标准步态轨迹生成功能
// ============================================================================

// 步态轨迹点结构
struct GaitTrajectoryPoint {
  float time;        // 相对时间（秒，0-周期时长）
  float hipAngle;    // 髋关节角度（度）
  float ankleAngle;  // 踝关节角度（度）
};

// 步态轨迹数据结构
struct GaitTrajectory {
  GaitTrajectoryPoint *points;  // 轨迹点数组
  uint16_t pointCount;           // 轨迹点数量
  float cycleDuration;           // 周期时长（秒）
  bool loaded;                   // 是否已加载数据
};

// 默认标准步态轨迹（基于采集的数据，简化版本）
// 这里先定义一个简单的测试轨迹，后续可以通过串口加载完整数据
#define MAX_GAIT_POINTS 200
GaitTrajectoryPoint defaultGaitPoints[MAX_GAIT_POINTS];
GaitTrajectory gaitTrajectory = {defaultGaitPoints, 0, 0.0f, false};

// 初始化默认步态轨迹（简单的正弦波测试轨迹）
void initDefaultGaitTrajectory() {
  // 创建一个简单的测试轨迹：髋关节和踝关节正弦波
  const uint16_t pointCount = 100;
  const float cycleDuration = 2.0f;  // 2秒周期
  
  for (uint16_t i = 0; i < pointCount; i++) {
    float phase = (float)i / (float)(pointCount - 1);  // 0.0 - 1.0
    float time = phase * cycleDuration;
    
    // 髋关节：-30度到30度的正弦波
    defaultGaitPoints[i].time = time;
    defaultGaitPoints[i].hipAngle = 30.0f * sin(phase * 2.0f * M_PI);
    
    // 踝关节：-10度到10度的正弦波，相位偏移90度
    defaultGaitPoints[i].ankleAngle = 10.0f * sin((phase + 0.25f) * 2.0f * M_PI);
  }
  
  gaitTrajectory.pointCount = pointCount;
  gaitTrajectory.cycleDuration = cycleDuration;
  gaitTrajectory.loaded = true;
  
  hostPrintln(">>> Default gait trajectory initialized");
}

// 步态数据接收状态
struct GaitDataReceiveState {
  bool receiving;           // 是否正在接收数据
  String jsonBuffer;         // JSON数据缓冲区
  uint32_t startTime;        // 开始接收时间
  uint32_t timeoutMs;        // 超时时间（毫秒）
};

GaitDataReceiveState gaitDataReceive = {false, "", 0, 5000};  // 5秒超时

// 加载步态数据（从JSON字符串）
bool loadGaitTrajectoryFromJson(const String &jsonStr) {
  // 使用ArduinoJson解析JSON
  // 动态分配内存，根据JSON字符串大小自动调整
  DynamicJsonDocument doc(16384);  // 最大16KB，足够处理200个数据点
  DeserializationError error = deserializeJson(doc, jsonStr);
  
  if (error) {
    hostPrintf(">>> Error parsing JSON: %s\n", error.c_str());
    return false;
  }
  
  // 检查必需字段
  if (!doc.containsKey("time") || !doc.containsKey("hip_angle") || !doc.containsKey("ankle_angle")) {
    hostPrintln(">>> Error: Missing required fields (time, hip_angle, ankle_angle)");
    return false;
  }
  
  JsonArray timeArray = doc["time"];
  JsonArray hipArray = doc["hip_angle"];
  JsonArray ankleArray = doc["ankle_angle"];
  
  uint16_t pointCount = timeArray.size();
  if (pointCount == 0 || pointCount > MAX_GAIT_POINTS) {
    hostPrintf(">>> Error: Invalid point count: %d (max: %d)\n", pointCount, MAX_GAIT_POINTS);
    return false;
  }
  
  if (hipArray.size() != pointCount || ankleArray.size() != pointCount) {
    hostPrintln(">>> Error: Array sizes don't match");
    return false;
  }
  
  // 获取周期时长（如果有）
  float cycleDuration = doc["cycle_duration"] | 0.0f;
  if (cycleDuration <= 0 && pointCount > 1) {
    // 如果没有提供周期时长，使用最后一个时间点
    cycleDuration = timeArray[pointCount - 1].as<float>();
  }
  
  // 加载数据到轨迹
  for (uint16_t i = 0; i < pointCount; i++) {
    defaultGaitPoints[i].time = timeArray[i].as<float>();
    defaultGaitPoints[i].hipAngle = hipArray[i].as<float>();
    defaultGaitPoints[i].ankleAngle = ankleArray[i].as<float>();
  }
  
  gaitTrajectory.pointCount = pointCount;
  gaitTrajectory.cycleDuration = cycleDuration;
  gaitTrajectory.loaded = true;
  
  hostPrintf(">>> Gait trajectory loaded: %d points, duration=%.2f s\n", 
                pointCount, cycleDuration);
  
  return true;
}

// 开始接收步态数据
void startReceivingGaitData() {
  gaitDataReceive.receiving = true;
  gaitDataReceive.jsonBuffer = "";
  gaitDataReceive.startTime = millis();
  hostPrintln(">>> Ready to receive gait data (JSON format)");
  hostPrintln(">>> Send JSON data now, timeout: 5 seconds");
}

// 停止接收步态数据
void stopReceivingGaitData() {
  gaitDataReceive.receiving = false;
  gaitDataReceive.jsonBuffer = "";
}

// 处理接收到的步态数据（在串口命令处理中调用）
void processReceivedGaitData(const String &line) {
  if (!gaitDataReceive.receiving) return;
  
  // 检查超时
  if (millis() - gaitDataReceive.startTime > gaitDataReceive.timeoutMs) {
    hostPrintln(">>> Timeout: Gait data reception timeout");
    stopReceivingGaitData();
    return;
  }
  
  // 累积数据
  gaitDataReceive.jsonBuffer += line;
  
  // 检查是否是完整的JSON（简单检查：以 { 开始，以 } 结束）
  int openBraces = 0;
  bool foundStart = false;
  int startIdx = -1;
  int endIdx = -1;
  
  for (int i = 0; i < gaitDataReceive.jsonBuffer.length(); i++) {
    char c = gaitDataReceive.jsonBuffer.charAt(i);
    if (c == '{') {
      if (!foundStart) {
        foundStart = true;
        startIdx = i;
      }
      openBraces++;
    } else if (c == '}') {
      openBraces--;
      if (openBraces == 0 && foundStart) {
        endIdx = i;
        break;
      }
    }
  }
  
  // 如果找到完整的JSON，解析并加载
  if (foundStart && endIdx >= 0 && openBraces == 0) {
    String jsonStr = gaitDataReceive.jsonBuffer.substring(startIdx, endIdx + 1);
    
    if (loadGaitTrajectoryFromJson(jsonStr)) {
      hostPrintln(">>> Gait trajectory loaded successfully!");
    } else {
      hostPrintln(">>> Failed to load gait trajectory");
    }
    
    stopReceivingGaitData();
  }
}

// 速度平滑器（限制加速度，确保速度曲线连续）
struct VelocitySmoother {
  float currentPosition;     // 当前位置
  float currentVelocity;     // 当前速度（dps）
  float maxAcceleration;     // 最大加速度（dps²）
  uint32_t lastUpdateMs;     // 上次更新时间
};

// 步态播放状态
struct GaitPlaybackState {
  bool active;              // 是否正在播放
  float frequency;           // 播放频率（Hz，即每秒多少个周期）
  float cycleDuration;       // 当前周期时长（秒）
  uint32_t cycleStartMs;     // 当前周期开始时间（毫秒）
  uint32_t lastUpdateMs;     // 上次更新时间
  uint32_t updateIntervalMs; // 更新间隔（毫秒）
  float currentPhase;        // 当前相位（0.0-1.0）
  float maxHipSpeedJoint;    // 髋关节最大速度限制（关节速度，dps）
  float maxAnkleSpeedJoint;   // 踝关节最大速度限制（关节速度，dps）
  float centerHipAngle;      // 摆动中心位置（髋关节）
  float centerAnkleAngle;    // 摆动中心位置（踝关节）
  VelocitySmoother hipSmoother;   // 髋关节速度平滑器
  VelocitySmoother ankleSmoother; // 踝关节速度平滑器
};

GaitPlaybackState gaitPlayback = {
  false, 1.0f, 2.0f, 0, 0, 5, 0.0f, 100.0f, 100.0f, 0.0f, 0.0f,
  {0.0f, 0.0f, 300.0f, 0},  // hipSmoother: 最大加速度300 dps²（关节速度）
  {0.0f, 0.0f, 300.0f, 0}   // ankleSmoother: 最大加速度300 dps²（关节速度）
};

// 线性插值函数
float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

// 平滑插值函数（使用平滑步进函数，避免速度突变）
// 使用smoothstep函数：t^2 * (3 - 2*t)，提供更平滑的过渡
float smoothLerp(float a, float b, float t) {
  // 将t限制在[0,1]范围内
  if (t <= 0.0f) return a;
  if (t >= 1.0f) return b;
  
  // 使用smoothstep函数进行平滑插值
  float smoothT = t * t * (3.0f - 2.0f * t);
  return a + (b - a) * smoothT;
}

// 速度平滑更新函数（限制加速度，确保速度曲线连续）
// 根据目标位置和最大加速度，平滑地更新当前位置和速度
float updateVelocitySmoother(VelocitySmoother &smoother, float targetPosition, uint32_t currentTimeMs) {
  if (smoother.lastUpdateMs == 0) {
    // 第一次调用，直接设置位置
    smoother.currentPosition = targetPosition;
    smoother.currentVelocity = 0.0f;
    smoother.lastUpdateMs = currentTimeMs;
    return targetPosition;
  }
  
  // 计算时间差（秒）
  float dt = (currentTimeMs - smoother.lastUpdateMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.1f) {
    // 时间差异常，直接设置位置
    smoother.currentPosition = targetPosition;
    smoother.currentVelocity = 0.0f;
    smoother.lastUpdateMs = currentTimeMs;
    return targetPosition;
  }
  
  // 计算位置误差
  float positionError = targetPosition - smoother.currentPosition;
  
  // 如果非常接近目标位置，平滑减速到停止
  const float stopThreshold = 0.05f;  // 5度以内开始减速
  if (fabsf(positionError) < stopThreshold) {
    // 平滑减速：速度按指数衰减
    smoother.currentVelocity *= 0.8f;  // 每次减少20%的速度
    if (fabsf(smoother.currentVelocity) < 0.1f) {
      smoother.currentVelocity = 0.0f;
      smoother.currentPosition = targetPosition;  // 直接到达目标
    } else {
      smoother.currentPosition += smoother.currentVelocity * dt;
    }
    smoother.lastUpdateMs = currentTimeMs;
    return smoother.currentPosition;
  }
  
  // 计算理想速度（如果直接到达目标位置需要的速度）
  float idealVelocity = positionError / dt;
  
  // 限制速度变化（加速度限制）
  float maxVelocityChange = smoother.maxAcceleration * dt;
  float velocityError = idealVelocity - smoother.currentVelocity;
  
  // 限制速度变化率（确保加速度连续）
  if (velocityError > maxVelocityChange) {
    smoother.currentVelocity += maxVelocityChange;
  } else if (velocityError < -maxVelocityChange) {
    smoother.currentVelocity -= maxVelocityChange;
  } else {
    smoother.currentVelocity = idealVelocity;
  }
  
  // 更新位置
  smoother.currentPosition += smoother.currentVelocity * dt;
  
  smoother.lastUpdateMs = currentTimeMs;
  return smoother.currentPosition;
}

// 根据相位获取步态轨迹点（使用线性插值）
void getGaitPointAtPhase(float phase, float &hipAngle, float &ankleAngle) {
  if (!gaitTrajectory.loaded || gaitTrajectory.pointCount == 0) {
    hipAngle = 0.0f;
    ankleAngle = 0.0f;
    return;
  }
  
  // 将相位转换为时间
  float targetTime = phase * gaitTrajectory.cycleDuration;
  
  // 找到对应的轨迹点（线性插值）
  if (targetTime <= gaitTrajectory.points[0].time) {
    hipAngle = gaitTrajectory.points[0].hipAngle;
    ankleAngle = gaitTrajectory.points[0].ankleAngle;
    return;
  }
  
  if (targetTime >= gaitTrajectory.points[gaitTrajectory.pointCount - 1].time) {
    uint16_t lastIdx = gaitTrajectory.pointCount - 1;
    hipAngle = gaitTrajectory.points[lastIdx].hipAngle;
    ankleAngle = gaitTrajectory.points[lastIdx].ankleAngle;
    return;
  }
  
  // 平滑插值查找（使用smoothLerp提供更平滑的过渡）
  for (uint16_t i = 0; i < gaitTrajectory.pointCount - 1; i++) {
    if (targetTime >= gaitTrajectory.points[i].time && 
        targetTime <= gaitTrajectory.points[i + 1].time) {
      float t = (targetTime - gaitTrajectory.points[i].time) / 
                (gaitTrajectory.points[i + 1].time - gaitTrajectory.points[i].time);
      // 使用平滑插值，避免速度突变
      hipAngle = smoothLerp(gaitTrajectory.points[i].hipAngle, 
                            gaitTrajectory.points[i + 1].hipAngle, t);
      ankleAngle = smoothLerp(gaitTrajectory.points[i].ankleAngle, 
                              gaitTrajectory.points[i + 1].ankleAngle, t);
      return;
    }
  }
}

// 根据轨迹和目标周期，自动计算所需的最大速度（关节速度，dps）
// 目标：在给定播放周期内尽可能完整地走完轨迹，而不是因为速度限制而缩小幅度
// 返回两个值：髋关节速度和踝关节速度（通过引用返回）
void computeRequiredMaxSpeed(float frequencyHz, float &hipSpeedJoint, float &ankleSpeedJoint) {
  if (!gaitTrajectory.loaded || gaitTrajectory.pointCount < 2 || frequencyHz <= 0.0f) {
    // 回退到默认值
    hipSpeedJoint = 100.0f;
    ankleSpeedJoint = 100.0f;
    return;
  }

  // 1. 计算轨迹本身（原始周期）上的最大关节速度（deg/s）
  float baseMaxHipVel = 0.0f;
  float baseMaxAnkleVel = 0.0f;

  for (uint16_t i = 0; i < gaitTrajectory.pointCount - 1; i++) {
    float t0 = gaitTrajectory.points[i].time;
    float t1 = gaitTrajectory.points[i + 1].time;
    float dt = t1 - t0;
    if (dt <= 0.0f) continue;

    float hip0 = gaitTrajectory.points[i].hipAngle;
    float hip1 = gaitTrajectory.points[i + 1].hipAngle;
    float ankle0 = gaitTrajectory.points[i].ankleAngle;
    float ankle1 = gaitTrajectory.points[i + 1].ankleAngle;

    float hipVel = fabsf(hip1 - hip0) / dt;     // deg/s（关节速度）
    float ankleVel = fabsf(ankle1 - ankle0) / dt; // deg/s（关节速度）

    if (hipVel > baseMaxHipVel) baseMaxHipVel = hipVel;
    if (ankleVel > baseMaxAnkleVel) baseMaxAnkleVel = ankleVel;
  }

  // 如果数据不合理，给一个默认速度
  if (baseMaxHipVel <= 0.0f) baseMaxHipVel = 100.0f;
  if (baseMaxAnkleVel <= 0.0f) baseMaxAnkleVel = 100.0f;

  // 2. 根据「原始周期」与「目标播放周期」的比例，放缩所需速度
  float baseCycle = gaitTrajectory.cycleDuration;
  if (baseCycle <= 0.0f) {
    // 如果没有配置周期，就用最后一个时间点近似
    baseCycle = gaitTrajectory.points[gaitTrajectory.pointCount - 1].time;
  }
  if (baseCycle <= 0.0f) {
    baseCycle = 1.0f;
  }

  float targetCycle = 1.0f / frequencyHz;        // 目标播放周期（秒）
  float speedScale = baseCycle / targetCycle;    // 周期缩放比：周期越短，speedScale 越大

  float requiredHip = baseMaxHipVel * speedScale;
  float requiredAnkle = baseMaxAnkleVel * speedScale;

  // 3. 加一点安全裕量，避免刚好打满导致跟随不上
  const float margin = 1.3f;
  requiredHip *= margin;
  requiredAnkle *= margin;

  // 4. 约束在安全范围内（避免速度设置过大）
  if (requiredHip < 30.0f) requiredHip = 30.0f;     // 太小会导致幅度不够
  if (requiredHip > 500.0f) requiredHip = 500.0f;    // 上限保护（关节速度）
  if (requiredAnkle < 30.0f) requiredAnkle = 30.0f;
  if (requiredAnkle > 500.0f) requiredAnkle = 500.0f;

  hipSpeedJoint = requiredHip;
  ankleSpeedJoint = requiredAnkle;
}

// 髋踝联动关系：根据髋角度计算踝角度（简化版本，后续可优化）
float calculateAnkleFromHip(float hipAngle) {
  // 这里使用简单的线性关系作为示例
  // 实际应该根据标准步态数据建立更精确的映射关系
  // 暂时返回0，使用轨迹数据中的踝角度
  return 0.0f;
}

// 轨迹平滑处理（移动平均滤波）
struct SmoothFilter {
  float *history;
  uint8_t size;
  uint8_t index;
  float sum;
};

float smoothValue(SmoothFilter &filter, float newValue) {
  if (filter.history == nullptr) return newValue;
  
  filter.sum -= filter.history[filter.index];
  filter.history[filter.index] = newValue;
  filter.sum += newValue;
  filter.index = (filter.index + 1) % filter.size;
  
  return filter.sum / filter.size;
}

// 平滑滤波器（髋关节和踝关节各一个）
#define SMOOTH_FILTER_SIZE 5
float hipSmoothHistory[SMOOTH_FILTER_SIZE] = {0};
float ankleSmoothHistory[SMOOTH_FILTER_SIZE] = {0};
SmoothFilter hipSmoothFilter = {hipSmoothHistory, SMOOTH_FILTER_SIZE, 0, 0.0f};
SmoothFilter ankleSmoothFilter = {ankleSmoothHistory, SMOOTH_FILTER_SIZE, 0, 0.0f};

// 启动步态轨迹播放
void startGaitPlayback(float frequencyHz, float maxSpeedDps) {
  if (!gaitTrajectory.loaded || gaitTrajectory.pointCount == 0) {
    hostPrintln("Error: Gait trajectory not loaded!");
    return;
  }

  // 如果未显式给出速度（或给的是非正数），根据轨迹和周期自动计算最大速度，
  // 目标是在给定周期内尽量走完完整的轨迹幅度。
  if (maxSpeedDps <= 0.0f) {
    computeRequiredMaxSpeed(frequencyHz, gaitPlayback.maxHipSpeedJoint, gaitPlayback.maxAnkleSpeedJoint);
    hostPrintf(">>> Auto max speed for gait playback (freq=%.2f Hz):\n", frequencyHz);
    hostPrintf(">>>   Hip joint speed: %.1f dps (motor shaft: %.1f dps)\n",
                  gaitPlayback.maxHipSpeedJoint,
                  jointSpeedToMotorSpeed(hipMotor, gaitPlayback.maxHipSpeedJoint));
    hostPrintf(">>>   Ankle joint speed: %.1f dps (motor shaft: %.1f dps)\n",
                  gaitPlayback.maxAnkleSpeedJoint,
                  jointSpeedToMotorSpeed(ankleMotor, gaitPlayback.maxAnkleSpeedJoint));
  } else {
    // 如果显式给出了速度，使用相同的速度（关节速度）
    gaitPlayback.maxHipSpeedJoint = maxSpeedDps;
    gaitPlayback.maxAnkleSpeedJoint = maxSpeedDps;
    hostPrintf(">>> Using specified joint speed: %.1f dps\n", maxSpeedDps);
  }
  
  // 读取当前角度作为摆动中心
  requestMotorAngle(hipMotor);
  requestMotorAngle(ankleMotor);
  delay(100);  // 等待CAN回复
  
  // 处理CAN消息，更新角度
  CAN_message_t inMsg;
  uint32_t startWait = millis();
  while (millis() - startWait < 200) {
    if (can1.read(inMsg)) {
      handleCanMessage(inMsg);
    }
  }
  
  // 保存当前位置作为摆动中心（使用逻辑角）
  gaitPlayback.centerHipAngle = getHipDeg();
  gaitPlayback.centerAnkleAngle = getAnkleDeg();
  
  // 初始化速度平滑器
  gaitPlayback.hipSmoother.currentPosition = getHipDeg();
  gaitPlayback.hipSmoother.currentVelocity = 0.0f;
  gaitPlayback.hipSmoother.lastUpdateMs = 0;  // 标记为未初始化
  
  gaitPlayback.ankleSmoother.currentPosition = getAnkleDeg();
  gaitPlayback.ankleSmoother.currentVelocity = 0.0f;
  gaitPlayback.ankleSmoother.lastUpdateMs = 0;  // 标记为未初始化
  
  gaitPlayback.active = true;
  gaitPlayback.frequency = frequencyHz;
  gaitPlayback.cycleDuration = 1.0f / frequencyHz;
  gaitPlayback.cycleStartMs = millis();
  gaitPlayback.currentPhase = 0.0f;
  
  hostPrintf(">>> Gait playback started: freq=%.2f Hz, duration=%.2f s\n",
                frequencyHz, gaitPlayback.cycleDuration);
  hostPrintf(">>> Center position: Hip=%.2f deg, Ankle=%.2f deg\n",
                gaitPlayback.centerHipAngle, gaitPlayback.centerAnkleAngle);
}

// 停止步态轨迹播放
void stopGaitPlayback() {
  gaitPlayback.active = false;
  hostPrintln(">>> Gait playback stopped");
}

// 更新步态轨迹播放
void updateGaitPlayback() {
  if (!gaitPlayback.active || !gaitTrajectory.loaded) return;
  
  uint32_t now = millis();
  if (now - gaitPlayback.lastUpdateMs < gaitPlayback.updateIntervalMs) return;
  
  gaitPlayback.lastUpdateMs = now;
  
  // 计算当前相位（0.0 - 1.0）
  uint32_t elapsedMs = now - gaitPlayback.cycleStartMs;
  float elapsedSec = elapsedMs / 1000.0f;
  
  gaitPlayback.currentPhase = fmod(elapsedSec, gaitPlayback.cycleDuration) / gaitPlayback.cycleDuration;
  
  // 获取当前相位对应的轨迹角度（相对于0度的偏移量）
  float trajectoryHipAngle, trajectoryAnkleAngle;
  getGaitPointAtPhase(gaitPlayback.currentPhase, trajectoryHipAngle, trajectoryAnkleAngle);
  
  // 将轨迹角度作为偏移量加到中心位置上
  float targetHipAngle = gaitPlayback.centerHipAngle + trajectoryHipAngle;
  float targetAnkleAngle = gaitPlayback.centerAnkleAngle + trajectoryAnkleAngle;
  
  // 使用速度平滑器更新位置（限制加速度，确保速度曲线连续）
  float smoothedHipAngle = updateVelocitySmoother(gaitPlayback.hipSmoother, targetHipAngle, now);
  float smoothedAnkleAngle = updateVelocitySmoother(gaitPlayback.ankleSmoother, targetAnkleAngle, now);
  
  // 计算实际需要的速度（基于平滑后的位置变化）
  // 注意：这里使用平滑器的速度，但需要转换为电机轴速度
  float hipVelocityJoint = fabsf(gaitPlayback.hipSmoother.currentVelocity);
  float ankleVelocityJoint = fabsf(gaitPlayback.ankleSmoother.currentVelocity);
  
  // 限制速度不超过最大值，并转换为电机轴速度
  if (hipVelocityJoint > gaitPlayback.maxHipSpeedJoint) {
    hipVelocityJoint = gaitPlayback.maxHipSpeedJoint;
  }
  if (ankleVelocityJoint > gaitPlayback.maxAnkleSpeedJoint) {
    ankleVelocityJoint = gaitPlayback.maxAnkleSpeedJoint;
  }
  
  uint16_t hipMotorSpeed = jointSpeedToMotorSpeed(hipMotor, hipVelocityJoint);
  uint16_t ankleMotorSpeed = jointSpeedToMotorSpeed(ankleMotor, ankleVelocityJoint);
  
  // 确保速度不为0（至少有一个最小值，避免电机停止）
  if (hipMotorSpeed < 10) hipMotorSpeed = 10;
  if (ankleMotorSpeed < 10) ankleMotorSpeed = 10;
  
  // 发送位置控制指令（带速度限制）
  sendPositionCommandWithSpeed(hipMotor, smoothedHipAngle, hipMotorSpeed);
  sendPositionCommandWithSpeed(ankleMotor, smoothedAnkleAngle, ankleMotorSpeed);
}

// ============================================================================
// 传感器轮询定时器（独立于数据采集，负责喂数据给状态机）
// ============================================================================

// 传感器轮询状态结构体已移动到文件头部
// struct SensorPollingTimer { ... };
// SensorPollingTimer sensorPolling = {false, 0, 20};


// 启动传感器轮询（在ctrlon开启时调用）
void startSensorPolling(uint32_t intervalMs = 20) {
  sensorPolling.enabled = true;
  sensorPolling.pollIntervalMs = intervalMs;
  sensorPolling.lastPollMs = millis();
  hostPrintf(">>> Sensor polling STARTED (interval: %lu ms, %.1f Hz)\n", 
                intervalMs, 1000.0f / intervalMs);
}

// 停止传感器轮询（在ctrloff时调用）
void stopSensorPolling() {
  sensorPolling.enabled = false;
  hostPrintln(">>> Sensor polling STOPPED");
}

// 更新传感器轮询（在loop中调用）
// 负责定期请求角度和状态2，喂数据给状态机
void updateSensorPolling() {
  if (!sensorPolling.enabled) return;
  // 如果发生系统错误，强制停止轮询
  if (isSystemError) {
    sensorPolling.enabled = false;
    return;
  }
  
  uint32_t now = millis();

  // 即使不发送轮询请求，也要尝试读取CAN消息，确保及时处理
  CAN_message_t inMsg;
  if (can1.read(inMsg)) {
    handleCanMessage(inMsg);
  }
  
  // 定期请求角度和状态（固定频率轮询）
  if (now - sensorPolling.lastPollMs >= sensorPolling.pollIntervalMs) {
    sensorPolling.lastPollMs = now;
    
    // 尝试发送请求，即使某一个失败也不返回，尽可能多发
    // 请求电机角度
    requestMotorAngle(ankleMotor);
    requestMotorAngle(hipMotor);
    
    // 请求踝关节状态2（获取电流数据）
    sendCanCommand(ankleMotor.id, CMD_READ_STATUS2, nullptr, 0, false);
    
    // 请求电机状态1（检测错误）
    sendCanCommand(ankleMotor.id, CMD_READ_STATUS1, nullptr, 0, false);
    sendCanCommand(hipMotor.id, CMD_READ_STATUS1, nullptr, 0, false);
  }
}

// ============================================================================
// 步态数据采集功能（只负责输出JSON，不再承担喂数据的职责）
// ============================================================================

// 步态数据采集状态
struct GaitDataCollection {
  bool enabled;              // 是否启用采集（只控制是否输出JSON）
  uint32_t lastSendMs;      // 上次发送数据的时间
  uint32_t sendIntervalMs;  // 发送间隔（毫秒），建议 10-50ms
};

// 4相步态实时输出状态（用于串口抓取最新相位数据）
struct Phase4RealtimeMonitor {
  bool enabled;
  uint32_t lastSendMs;
  uint32_t sendIntervalMs;
};

Phase4RealtimeMonitor phase4Monitor = {false, 0, 100};  // 默认 10Hz

// 控制循环状态结构体已移动到文件头部
// struct ControlLoop { ... };
// extern ControlLoop controlLoop;


// ============================================================================
// 转矩助力控制：参数与状态（最小侵入式新增）
// ============================================================================

// 助力参数
struct TorqueAssistParams {
  // ankle swing DF
  float theta_min_df = 5.0f;
  float theta_margin = 2.0f;
  float e_dead = 2.0f;
  float e_sat = 10.0f;
  float swing_df_start = 0.05f;
  float swing_df_end   = 0.40f;
  float swing_unload_s = 0.60f;
  int16_t iq_df_max = 5;   // -DF（本机型：负数为背屈）
  // push-off / A1：产品门控以 4 相 PHASE4_PUSH_OFF 为准（见 computeAnkleIqTarget）
  float stance_pf_start = 0.75f;  // 已废弃：旧版 stance_pct 窗，仅保留兼容串口/调参占位
  float stance_pf_end   = 0.95f;
  float hip_ext_th      = -6.0f;  // 原-8，放宽髋伸展要求
  float ankle_df_th     = 18.0f;  // 原22，降低背屈阈值以增加触发机会
  uint32_t tpulse_ms    = 120;    // 已废弃：由 pushoff_max_ms 与踝角结束条件替代
  float ankle_pf_target_deg = 10.0f;  // 背屈为正；踝角 ≤ 此值则结束跖屈助力（须 < ankle_df_th）
  uint32_t pushoff_max_ms = 300;     // 跖屈助力持续时间兜底（ms）
  int16_t iq_pf_max     = 1200;    // +PF （本机型：正数为跖屈）；实测800有感，调至1200+
  int16_t iq_pf_floor   = 800;     // +PF 起步偏置；实测<800无感，直接从800起步
  // hip 助力窗口与幅值（可通过串口/上位机调节）
  // hipAssistWindowEnd: 髋关节助力在 SWING 早期持续的摆动进度上限（0~1），例如 0.4 表示前 40%
  // hipAssistMaxIq:     髋关节助力的最大峰值电流（iq 单位，可映射到 mA），例如 1500mA
  float  hipAssistWindowEnd = 0.4f;
  int16_t hipAssistMaxIq    = 50;
  // 仍保留原有 hip 斜率/限幅参数，用于安全管线
  float hip_win_start   = 0.0f;
  float hip_win_end     = 0.25f;
  int16_t iq_hip_flex_max = 10;
  // slew per 10ms (OPTIMIZED FOR PUSH-OFF)
  int16_t diq_up_df  = 4,   diq_dn_df  = 8;   // DF上升
  int16_t diq_up_pf  = 500, diq_dn_pf  = 20;  // PF斜率大幅提速：500/10ms，约20~30ms内到达峰值
  int16_t diq_up_hip = 3,   diq_dn_hip = 6;   // 髋保持不变
  // abnormal
  int16_t iq_small   = 20;
  float   v_rev      = 5.0f;
  uint32_t t_no_move_ms = 200;
  float   v_small    = 3.0f;
  float   a_small    = 0.5f;
  uint32_t cooldown_ms  = 500;
  uint32_t soft_exit_ms = 200;
  // stance progress
  float Tst_init = 0.6f; // 初始 stance 周期
  // 安全管线旁路（测试用）：跳过斜率限幅和 compliant/cooldown，直接下发 iq_target（仅幅值限幅保留）
  bool ankleBypassSafety = false;
};
TorqueAssistParams torqueParams;

// A1 可调参数 EEPROM 持久化（上电自动加载）
struct A1ParamsPersist {
  uint32_t magic;
  uint16_t version;
  float ankle_df_th;
  float hip_ext_th;
  uint32_t pushoff_max_ms;
  float ankle_pf_target_deg;
  int16_t iq_pf_max;
  int16_t iq_pf_floor;
  int16_t diq_up_pf;
  uint8_t  ankleBypassSafety;  // v2 新增：0=安全管线，1=旁路测试
  uint8_t  _pad;               // 对齐到偶数字节
  uint16_t checksum;
};

static const int EEPROM_ADDR_A1_PARAMS = 0;
static const uint32_t A1_PARAMS_MAGIC = 0x41315052UL;  // "A1PR"
static const uint16_t A1_PARAMS_VERSION = 2;  // v2: 加入 ankleBypassSafety 字段

uint16_t calcA1ParamsChecksum(const A1ParamsPersist &p) {
  uint16_t c = 0x5A5A;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&p);
  for (size_t i = 0; i < sizeof(A1ParamsPersist) - sizeof(p.checksum); ++i) {
    c = (uint16_t)(c + bytes[i] * (uint16_t)(i + 1));
  }
  return c;
}

void saveA1ParamsToEeprom() {
  A1ParamsPersist p;
  p.magic = A1_PARAMS_MAGIC;
  p.version = A1_PARAMS_VERSION;
  p.ankle_df_th = torqueParams.ankle_df_th;
  p.hip_ext_th = torqueParams.hip_ext_th;
  p.pushoff_max_ms = torqueParams.pushoff_max_ms;
  p.ankle_pf_target_deg = torqueParams.ankle_pf_target_deg;
  p.iq_pf_max = torqueParams.iq_pf_max;
  p.iq_pf_floor = torqueParams.iq_pf_floor;
  p.diq_up_pf = torqueParams.diq_up_pf;
  p.ankleBypassSafety = torqueParams.ankleBypassSafety ? 1u : 0u;
  p._pad = 0;
  p.checksum = calcA1ParamsChecksum(p);
  EEPROM.put(EEPROM_ADDR_A1_PARAMS, p);
  hostPrintln(">>> A1 params saved to EEPROM");
}

bool loadA1ParamsFromEeprom() {
  A1ParamsPersist p;
  EEPROM.get(EEPROM_ADDR_A1_PARAMS, p);
  if (p.magic != A1_PARAMS_MAGIC || p.version != A1_PARAMS_VERSION) {
    return false;
  }
  uint16_t crc = calcA1ParamsChecksum(p);
  if (crc != p.checksum) {
    hostPrintln("WARN: EEPROM A1 params checksum mismatch, using defaults");
    return false;
  }

  // 范围和关联校验，防止写入损坏数据
  if (p.ankle_df_th < 0.0f || p.ankle_df_th > 40.0f) return false;
  if (p.hip_ext_th < -40.0f || p.hip_ext_th > 20.0f) return false;
  if (p.pushoff_max_ms < 50 || p.pushoff_max_ms > 1000) return false;
  if (p.ankle_pf_target_deg < -10.0f || p.ankle_pf_target_deg > 30.0f) return false;
  if (p.ankle_pf_target_deg >= p.ankle_df_th) return false;
  if (p.iq_pf_max < 1 || p.iq_pf_max > 2000) return false;
  if (p.iq_pf_floor < 0 || p.iq_pf_floor > 2000) return false;
  if (p.iq_pf_floor > p.iq_pf_max) return false;
  if (p.diq_up_pf < 1 || p.diq_up_pf > 1000) return false;

  torqueParams.ankle_df_th = p.ankle_df_th;
  torqueParams.hip_ext_th = p.hip_ext_th;
  torqueParams.pushoff_max_ms = p.pushoff_max_ms;
  torqueParams.ankle_pf_target_deg = p.ankle_pf_target_deg;
  torqueParams.iq_pf_max = p.iq_pf_max;
  torqueParams.iq_pf_floor = p.iq_pf_floor;
  torqueParams.diq_up_pf = p.diq_up_pf;
  torqueParams.ankleBypassSafety = (p.ankleBypassSafety != 0);
  return true;
}

// 全局髋关节助力最大电流（mA 或 驱动单位），可通过串口或上位机调整
int16_t hipAssistMaxIq = 0; // 0 表示使用 torqueParams.hipAssistMaxIq

// 控制杆输入（需在串口/CAN 回调中更新）
volatile int16_t ankle_cmd_amp = 0; // 踝 push-off / DF 强度
volatile int16_t hip_cmd_amp   = 0; // 髋屈助力强度

// 踝速度估计
struct AnkleVelEstimator {
  bool initialized = false;
  float last_deg = 0.0f;
  uint32_t last_ms = 0;
  float vel = 0.0f;
  float vel_f = 0.0f;   // EMA 滤波
};
AnkleVelEstimator ankleVel;

// STANCE 进度估计
struct StanceProgress {
  float Tst_avg = 0.6f;
  uint32_t phase_start_ms = 0;
};
StanceProgress stanceProg;

// Push-off 脉冲状态机
struct PushOffPulse {
  bool active = false;
  uint32_t start_ms = 0;
  int16_t iq_peak = 0; // 负值（跖屈）
};
PushOffPulse pushOff;

// 关节安全状态（斜率限制 + 冷却）
struct JointSafetyState {
  int16_t iq_cmd_prev = 0;
  bool compliant = false;
  bool in_cooldown = false;
  uint32_t cooldown_start_ms = 0;
  uint32_t abnormal_start_ms = 0;
};
JointSafetyState ankleSafety;
JointSafetyState hipSafety;

struct AssistDebugFlags {
  bool pushOffActive = false;
  bool dfActive = false;
  bool unloadActive = false;
};
AssistDebugFlags assistFlags;

// 异常原因（仅用于调试）
enum AbnormalReason {
  ABN_NONE = 0,
  ABN_REV_DIR,
  ABN_NO_MOVE
};
AbnormalReason ankleAbn = ABN_NONE;

// ============================================================================
// 辅助函数：速度估计 / STANCE 进度 / 安全管线 / IQ 计算
// ============================================================================

void updateAnkleVelEstimator(float ankle_deg, uint32_t nowMs) {
  if (!ankleVel.initialized) {
    ankleVel.initialized = true;
    ankleVel.last_deg = ankle_deg;
    ankleVel.last_ms = nowMs;
    ankleVel.vel = 0.0f;
    ankleVel.vel_f = 0.0f;
    return;
  }
  uint32_t dt_ms = nowMs - ankleVel.last_ms;
  if (dt_ms == 0) return;
  float dt = dt_ms / 1000.0f;
  float vel = (ankle_deg - ankleVel.last_deg) / dt;
  ankleVel.vel = vel;
  const float beta = 0.2f; // 与 hip 速度滤波一致
  ankleVel.vel_f = ankleVel.vel_f + beta * (vel - ankleVel.vel_f);
  ankleVel.last_deg = ankle_deg;
  ankleVel.last_ms = nowMs;
}

void updateStanceProgress(GaitPhase phase, uint32_t nowMs) {
  static GaitPhase lastPhase = PHASE_STANCE;
  if (stanceProg.Tst_avg <= 0.05f) stanceProg.Tst_avg = torqueParams.Tst_init;

  if (phase != lastPhase) {
    if (lastPhase == PHASE_STANCE && phase == PHASE_SWING) {
      uint32_t dur_ms = nowMs - stanceProg.phase_start_ms;
      float Tnew = dur_ms / 1000.0f;
      if (Tnew > 0.1f && Tnew < 3.0f) {
        stanceProg.Tst_avg = 0.8f * stanceProg.Tst_avg + 0.2f * Tnew;
      }
    }
    stanceProg.phase_start_ms = nowMs;
    lastPhase = phase;
  }
}

float getStancePct(GaitPhase phase, uint32_t nowMs) {
  if (phase != PHASE_STANCE) return 0.0f;
  uint32_t dur_ms = nowMs - stanceProg.phase_start_ms;
  float Tst = (stanceProg.Tst_avg > 0.1f) ? stanceProg.Tst_avg : torqueParams.Tst_init;
  float pct = (dur_ms / 1000.0f) / Tst;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 1.0f) pct = 1.0f;
  return pct;
}

int16_t applySafetyPipeline(JointSafetyState &st,
                            int16_t iq_target,
                            int16_t iq_pos_max,
                            int16_t iq_neg_max,
                            int16_t diq_up,
                            int16_t diq_dn,
                            uint32_t nowMs) {
  // 冷却期：强制 0
  if (st.in_cooldown) {
    if (nowMs - st.cooldown_start_ms >= torqueParams.cooldown_ms) {
      st.in_cooldown = false;
    } else {
      iq_target = 0;
    }
  }

  // 幅值限幅（正向 PF / 负向 DF）
  if (iq_target > iq_pos_max) iq_target = iq_pos_max;
  if (iq_target < -iq_neg_max) iq_target = -iq_neg_max;

  int16_t iq_prev = st.iq_cmd_prev;
  int16_t diff = iq_target - iq_prev;
  if (diff > diq_up) diff = diq_up;
  if (diff < -diq_dn) diff = -diq_dn;
  int16_t iq_cmd = iq_prev + diff;

  // 软退出：把 iq 拉回 0，并进入冷却
  if (st.compliant) {
    if (iq_cmd > 0) {
      iq_cmd -= torqueParams.diq_dn_df;
      if (iq_cmd < 0) iq_cmd = 0;
    } else if (iq_cmd < 0) {
      iq_cmd += torqueParams.diq_dn_df;
      if (iq_cmd > 0) iq_cmd = 0;
    }
    if (iq_cmd == 0) {
      st.compliant = false;
      st.in_cooldown = true;
      st.cooldown_start_ms = nowMs;
    }
  }

  st.iq_cmd_prev = iq_cmd;
  return iq_cmd;
}

void updateAnkleAbnormalDetector(int16_t iq_cmd,
                                 uint32_t nowMs,
                                 float ankle_deg,
                                 float ankle_deg_prev,
                                 float ankle_vel_f) {
  static uint32_t rev_counter_ms = 0;
  static uint32_t no_move_start_ms = 0;
  static float last_deg_for_no_move = 0.0f;

  ankleAbn = ABN_NONE;

  // D1: 速度反向（50ms 确认）
  if (abs(iq_cmd) > torqueParams.iq_small) {
    bool expect_df = iq_cmd < 0;
    if (expect_df && ankle_vel_f < -torqueParams.v_rev) {
      rev_counter_ms += 10;
    } else if (!expect_df && ankle_vel_f > torqueParams.v_rev) {
      rev_counter_ms += 10;
    } else {
      rev_counter_ms = 0;
    }
    if (rev_counter_ms >= 50) {
      ankleAbn = ABN_REV_DIR;
    }
  } else {
    rev_counter_ms = 0;
  }

  // D2: 无响应卡滞（200ms 窗）
  if (abs(iq_cmd) > torqueParams.iq_small) {
    if (no_move_start_ms == 0) {
      no_move_start_ms = nowMs;
      last_deg_for_no_move = ankle_deg;
    } else if (nowMs - no_move_start_ms >= torqueParams.t_no_move_ms) {
      float ddeg = fabsf(ankle_deg - last_deg_for_no_move);
      if (fabsf(ankle_vel_f) < torqueParams.v_small &&
          ddeg < torqueParams.a_small) {
        ankleAbn = ABN_NO_MOVE;
      }
      no_move_start_ms = nowMs;
      last_deg_for_no_move = ankle_deg;
    }
  } else {
    no_move_start_ms = 0;
  }
}

int16_t computeAnkleIqTarget(GaitPhase phase,
                             float swing_pct,
                             float stance_pct,
                             float ankle_deg,
                             float /*ankle_vel_f*/,
                             float hip_deg,
                             uint32_t nowMs) {
  assistFlags.pushOffActive = false;
  assistFlags.dfActive = false;
  assistFlags.unloadActive = false;

  int16_t iq_target = 0;

  // A3: Swing late unload
  if (phase == PHASE_SWING && swing_pct > torqueParams.swing_unload_s) {
    assistFlags.unloadActive = true;
    return 0;
  }

  // A1: Push-off（仅 STANCE；触发门控 = 4 相 PHASE4_PUSH_OFF，非退化）
  if (phase == PHASE_STANCE) {
    (void)stance_pct; // 产品逻辑不再使用 stance 进度窗
    const GaitPhase4 ph4 = getCurrentGaitPhase4();
    const bool pf_ok = !isPhase4Degraded();
    const bool in_pushoff_phase = (ph4 == PHASE4_PUSH_OFF);
    const bool hip_ok = hip_deg <= torqueParams.hip_ext_th;
    const bool ankle_ok = ankle_deg >= torqueParams.ankle_df_th;

    if (pushOff.active) {
      const bool lift_off = (ph4 == PHASE4_SWING);  // 此处 phase 恒为 STANCE
      const bool left_p3 = (ph4 != PHASE4_PUSH_OFF);
      const bool angle_end = ankle_deg <= torqueParams.ankle_pf_target_deg;
      const bool timeout = (nowMs - pushOff.start_ms) >= torqueParams.pushoff_max_ms;
      if (lift_off || left_p3 || !pf_ok || angle_end || timeout) {
        pushOff.active = false;
      }
    }

    if (!pushOff.active && in_pushoff_phase && pf_ok && hip_ok && ankle_ok &&
        !ankleSafety.in_cooldown) {
      pushOff.active = true;
      pushOff.start_ms = nowMs;
      int16_t peak = (int16_t)abs(ankle_cmd_amp);
      if (peak <= 0) peak = torqueParams.iq_pf_max;
      if (peak > torqueParams.iq_pf_max) peak = torqueParams.iq_pf_max;
      pushOff.iq_peak = peak; // 跖屈为正（本机型）
    }

    if (pushOff.active && !ankleSafety.in_cooldown) {
      // A1 平滑偏置：目标至少为 floor，避免 0~300 区间“无感”
      int16_t floorVal = torqueParams.iq_pf_floor;
      if (floorVal < 0) floorVal = 0;
      if (floorVal > torqueParams.iq_pf_max) floorVal = torqueParams.iq_pf_max;
      iq_target = pushOff.iq_peak;
      if (iq_target < floorVal) {
        iq_target = floorVal;
      }
      assistFlags.pushOffActive = true;
    }
    return iq_target; // STANCE 阶段不做 swing DF
  }

  // 进入 SWING 时，如脉冲未结束则强制关闭（与离撑一致）
  if (phase == PHASE_SWING && pushOff.active) {
    pushOff.active = false;
  }

  // A2: Swing early DF（与 4 相 PHASE4_SWING 对齐；退化模式下关闭）
  if (phase == PHASE_SWING &&
      getCurrentGaitPhase4() == PHASE4_SWING &&
      !isPhase4Degraded() &&
      swing_pct >= torqueParams.swing_df_start &&
      swing_pct <= torqueParams.swing_df_end) {
    float theta_min_df = torqueParams.theta_min_df;
    float margin = torqueParams.theta_margin;
    float e = theta_min_df - ankle_deg;
    float e_eff = e - torqueParams.e_dead;
    if (e_eff < 0.0f) e_eff = 0.0f;
    if (e_eff > torqueParams.e_sat) e_eff = torqueParams.e_sat;

    if (e_eff > 0.0f) {
      float Kdf = (float)torqueParams.iq_df_max / torqueParams.e_sat;
      float iqf = Kdf * e_eff;
      if (iqf > torqueParams.iq_df_max) iqf = (float)torqueParams.iq_df_max;
      iq_target = (int16_t)(-iqf); // DF 负向（本机型）
      assistFlags.dfActive = true;
    } else if (ankle_deg > theta_min_df + margin) {
      iq_target = 0;
    }
  }

  return iq_target;
}

int16_t computeHipIqTarget(GaitPhase phase,
                           float swing_pct,
                           float hip_deg,
                           float hip_vel_f) {
  (void)hip_deg;
  (void)hip_vel_f;
  // 仅在摆动相（SWING）内提供助力
  if (phase != PHASE_SWING) {
    return 0;
  }

  // 助力窗口：仅在 SWING 早期 [0, hipAssistWindowEnd] 内生效
  float winEnd = torqueParams.hipAssistWindowEnd;
  if (winEnd <= 0.0f) {
    return 0;
  }
  if (swing_pct < 0.0f || swing_pct > winEnd) {
    return 0;
  }

  // 将全局摆动进度映射到 [0,1] 的局部归一化时间 u
  // u = 0: 助力开始；u = 1: 助力结束
  float u = swing_pct / winEnd;
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;

  // 使用半个正弦波作为平滑窗函数：
  // window(u) = sin(pi * u), 在 u ∈ [0,1] 上从 0 平滑上升到 1 再平滑回到 0
  const float PI_F = 3.1415926f;
  float window = sinf(PI_F * u);
  if (window < 0.0f) window = 0.0f;  // 理论上不会小于0，这里只是防御性处理

  // 助力峰值：由控制杆 hip_cmd_amp 决定，并受 hipAssistMaxIq 限幅
    // 全局可调节的最大助力强度（以 hipAssistMaxIq 为优先）
    extern int16_t hipAssistMaxIq;
    int16_t maxIq = (hipAssistMaxIq > 0) ? hipAssistMaxIq : torqueParams.hipAssistMaxIq;
  if (maxIq <= 0) {
    // 若未显式配置，则退回到原有的 iq_hip_flex_max
    maxIq = torqueParams.iq_hip_flex_max;
  }

  int16_t amp = (int16_t)abs(hip_cmd_amp);
  if (amp <= 0) {
    amp = maxIq;
  } else if (amp > maxIq) {
    amp = maxIq;
  }

  // 髋屈为正方向，若实际机械安装相反，可将 sign 改为 -1
  int16_t sign = 1;

  // 根据窗函数缩放得到当前时刻的目标电流
  float iq_f = (float)amp * window;
  int16_t iq = (int16_t)iq_f;
  return sign * iq;
}

struct AssistDebugSnapshot {
  int phase = 0;
  float swing_pct = 0.0f;
  float stance_pct = 0.0f;
  float ankle_deg = 0.0f;
  float ankle_vel_f = 0.0f;
  int16_t ankle_iq_target = 0;
  int16_t ankle_iq_cmd = 0;
  float hip_deg = 0.0f;
  float hip_vel_f = 0.0f;
  int16_t hip_iq_target = 0;
  int16_t hip_iq_cmd = 0;
  uint8_t pf = 0, df = 0, ul = 0;
  uint8_t comp = 0, cool = 0, abn = 0;
  // 4相快照
  int     phase4 = 0;
  float   phase4_progress = 0.0f;
  float   phase4_output = 0.0f;
  uint8_t phase4_degraded = 0;
  uint32_t lastUpdateMs = 0;
};

AssistDebugSnapshot assistDbg;  // 控制循环内更新，sendGaitData/loop 中读取

GaitDataCollection gaitCollection = {false, 0, 20}; // 默认20ms间隔（50Hz）

// 发送步态数据到串口（固定 JSON schema，便于上位机稳定解析）
void sendGaitData() {
  const uint32_t now = millis();
  const float h = getHipDeg();
  const float a = getAnkleDeg();

  // 蓝牙模式先发送精简字段，降低带宽压力
  if (useBluetoothTelemetry) {
    const int ph = phase4Det.initialized ? ((int)getCurrentGaitPhase4() * 10) : 0;
    const int iqC_a = (int)assistDbg.ankle_iq_cmd;
    const int iqC_h = (int)assistDbg.hip_iq_cmd;
    telemetryPrintf("{\"t\":%lu,\"h\":%.2f,\"ank\":%.2f,\"ph\":%d,\"iqC_a\":%d,\"iqC_h\":%d}\n",
                    now, h, a, ph, iqC_a, iqC_h);
    return;
  }

  const float hf = hipProcessor.initialized ? hipProcessor.hip_f : h;
  const float hvf = hipProcessor.initialized ? hipProcessor.hip_vel_f : 0.0f;
  const int phase = gaitPhaseDetector.initialized ? (int)gaitPhaseDetector.currentPhase : 0;
  const float s = swingProgress.initialized ? swingProgress.swing_progress : 0.0f;
  const float ar = ankleAssist.initialized ? getAnkleReferenceAngle() : a;
  const int act = controlLoop.anklePositionActive ? 1 : 0;
  const float hm = gaitPhaseDetector.initialized ? gaitPhaseDetector.hip_max_last : 0.0f;

  const int ph4 = phase4Det.initialized ? (int)getCurrentGaitPhase4() : 0;
  const int ph4v = ph4 * 10;  // 0/10/20/30：用于上位机阶梯显示
  const float ph4p = phase4Det.initialized ? getPhase4Progress() : 0.0f;
  const float ph4o = phase4Det.initialized ? getPhase4ProfileOutput() : 0.0f;
  const int ph4d = phase4Det.initialized ? (isPhase4Degraded() ? 1 : 0) : 0;

  // 理论助力链路（当前阶段用于观察，不代表已下发电机）
  const int iqT_a = (int)assistDbg.ankle_iq_target;
  const int iqC_a = (int)assistDbg.ankle_iq_cmd;
  const int iqT_h = (int)assistDbg.hip_iq_target;
  const int iqC_h = (int)assistDbg.hip_iq_cmd;
  const int PF = (int)assistDbg.pf;
  const int DF = (int)assistDbg.df;
  const int UL = (int)assistDbg.ul;
  const int comp = (int)assistDbg.comp;
  const int cool = (int)assistDbg.cool;
  const int abn = (int)assistDbg.abn;
  const float st = assistDbg.stance_pct;
  const float ank = assistDbg.ankle_deg;
  const float v = assistDbg.ankle_vel_f;
  const float hip = assistDbg.hip_deg;
  const float hipv = assistDbg.hip_vel_f;
  const int ph = ph4v;  // 上位机现有 "ph" 曲线直接显示四相放大值

  telemetryPrintf(
      "{\"t\":%lu,\"h\":%.2f,\"hf\":%.2f,\"hvf\":%.2f,\"vf\":%.2f,"
      "\"phase\":%d,\"s\":%.3f,\"a\":%.2f,\"ar\":%.2f,\"act\":%d,\"hm\":%.2f,"
      "\"ph4\":%d,\"ph4v\":%d,\"ph4p\":%.3f,\"ph4o\":%.3f,\"ph4d\":%d,"
      "\"ph\":%d,\"st\":%.3f,\"ank\":%.2f,\"v\":%.2f,\"hip\":%.2f,\"hipv\":%.2f,"
      "\"iqT_a\":%d,\"iqC_a\":%d,\"iqT_h\":%d,\"iqC_h\":%d,"
      "\"PF\":%d,\"DF\":%d,\"UL\":%d,\"comp\":%d,\"cool\":%d,\"abn\":%d}\n",
      now, h, hf, hvf, hvf,
      phase, s, a, ar, act, hm,
      ph4, ph4v, ph4p, ph4o, ph4d,
      ph, st, ank, v, hip, hipv,
      iqT_a, iqC_a, iqT_h, iqC_h,
      PF, DF, UL, comp, cool, abn);
}

// 启动/停止步态数据采集（只控制是否输出JSON）
void startGaitCollection(uint32_t intervalMs = 20) {
  gaitCollection.enabled = true;
  gaitCollection.sendIntervalMs = intervalMs;
  gaitCollection.lastSendMs = 0;
  hostPrintln(">>> Gait data collection STARTED (JSON output only)");
  hostPrintf(">>> Output interval: %lu ms (%.1f Hz)\n", 
                intervalMs, 1000.0f / intervalMs);
  hostPrintln(">>> Note: Sensor polling should be enabled separately (via ctrlon)");
  startSensorPolling(intervalMs);
}

void stopGaitCollection() {
  gaitCollection.enabled = false;
  hostPrintln(">>> Gait data collection STOPPED");
  stopSensorPolling();
}

// 更新步态数据采集（在loop中调用）
// 只负责输出JSON，不再承担喂数据的职责
void updateGaitCollection() {
  if (!gaitCollection.enabled) return;
  
  uint32_t now = millis();
  
  // 定期发送数据到串口（如果距离上次发送超过间隔，且有新数据）
  if (now - gaitCollection.lastSendMs >= gaitCollection.sendIntervalMs) {
    // 检查数据是否更新（避免发送旧数据）
    if (hipStatus.lastUpdateMs > 0 && ankleStatus.lastUpdateMs > 0) {
      gaitCollection.lastSendMs = now;
      sendGaitData();
    }
  }
}

void sendPhase4RealtimeData() {
  uint32_t now = millis();
  int phase4 = phase4Det.initialized ? (int)phase4Det.currentPhase : -1;
  int basePhase = gaitPhaseDetector.initialized ? (int)gaitPhaseDetector.currentPhase : -1;
  float stancePct = gaitPhaseDetector.initialized ?
      getStancePct(gaitPhaseDetector.currentPhase, now) : 0.0f;
  uint32_t phaseDurMs = phase4Det.initialized ? (now - phase4Det.phaseStartMs) : 0;
  uint32_t degradedMs = (phase4Det.initialized && phase4Det.degraded) ?
      (now - phase4Det.degradedStartMs) : 0;

  // 结构化输出，便于上位机/脚本直接抓取
  telemetryPrintf("{\"ph4rt\":1,\"t\":%lu,\"init\":%d,\"ph4\":%d,\"p\":%.3f,"
                "\"out\":%.3f,\"deg\":%d,\"dur\":%lu,\"deg_ms\":%lu,"
                "\"base\":%d,\"stance\":%.3f,\"trans\":%d}\n",
                now,
                phase4Det.initialized ? 1 : 0,
                phase4,
                phase4Det.initialized ? phase4Det.phaseProgress : 0.0f,
                phase4Det.initialized ? phase4Det.profileOutput : 0.0f,
                phase4Det.degraded ? 1 : 0,
                phaseDurMs,
                degradedMs,
                basePhase,
                stancePct,
                phase4Det.transitionCount);
}

void updatePhase4RealtimeMonitor() {
  if (!phase4Monitor.enabled) return;
  uint32_t now = millis();
  if (now - phase4Monitor.lastSendMs >= phase4Monitor.sendIntervalMs) {
    phase4Monitor.lastSendMs = now;
    sendPhase4RealtimeData();
  }
}

// ============================================================================
// 主循环控制函数前向声明
// ============================================================================

// 控制循环变量（定义已在前文声明，这里仅前向声明供命令处理使用）
extern ControlLoop controlLoop;

void setControlLoopEnabled(bool enabled);
void updateControlLoop();

// TS系列 0xA1/0x9C 的 iq 单位换算：66/4096 A per LSB
float iqLsbToAmpTs(int16_t iqLsb) {
  return ((float)iqLsb) * 66.0f / 4096.0f;
}

void queryAndPrintAnkleIqTs() {
  // 主动请求一次踝关节状态2（0x9C），并等待短时间处理回包
  sendCanCommand(ankleMotor.id, CMD_READ_STATUS2, nullptr, 0, false);
  uint32_t t0 = millis();
  while (millis() - t0 < 80) {
    CAN_message_t inMsg;
    while (can1.read(inMsg)) {
      handleCanMessage(inMsg);
    }
    if (ankleStatus.lastUpdateMs > 0 && (millis() - ankleStatus.lastUpdateMs) < 120) {
      break;
    }
    delay(5);
  }

  int16_t iqRaw = ankleStatus.iq;
  float iqA = iqLsbToAmpTs(iqRaw);
  hostPrintf(">>> Ankle iq(raw)=%d LSB, TS est current=%.3f A (%.1f mA)\n",
                (int)iqRaw, iqA, iqA * 1000.0f);
}

void printA1Params() {
  hostPrintln(">>> A1 tunable params:");
  hostPrintf(">>>   ankle_df_th=%.2f deg (range: 0.0~40.0)\n", torqueParams.ankle_df_th);
  hostPrintf(">>>   hip_ext_th=%.2f deg (range: -40.0~20.0)\n", torqueParams.hip_ext_th);
  hostPrintf(">>>   pushoff_max_ms=%lu ms (range: 50~1000)\n", (unsigned long)torqueParams.pushoff_max_ms);
  hostPrintf(">>>   ankle_pf_target_deg=%.2f deg (range: -10.0~30.0, must < ankle_df_th)\n",
                torqueParams.ankle_pf_target_deg);
  hostPrintf(">>>   iq_pf_max=%d (range: 1~2000)\n", (int)torqueParams.iq_pf_max);
  hostPrintf(">>>   iq_pf_floor=%d (range: 0~2000, must <= iq_pf_max)\n", (int)torqueParams.iq_pf_floor);
  hostPrintf(">>>   diq_up_pf=%d (range: 1~1000)\n", (int)torqueParams.diq_up_pf);
  hostPrintf(">>>   ankleBypassSafety=%d (0=安全管线, 1=旁路/测试)\n", torqueParams.ankleBypassSafety ? 1 : 0);
}

bool setA1Param(const String& name, const String& valueStr) {
  if (name == "ankle_df_th") {
    float v = valueStr.toFloat();
    if (v < 0.0f || v > 40.0f) {
      hostPrintln("ERROR: ankle_df_th out of range (0.0~40.0)");
      return false;
    }
    if (torqueParams.ankle_pf_target_deg >= v) {
      hostPrintln("ERROR: ankle_df_th must be greater than ankle_pf_target_deg");
      return false;
    }
    torqueParams.ankle_df_th = v;
    hostPrintf(">>> Set ankle_df_th=%.2f deg\n", torqueParams.ankle_df_th);
    return true;
  }
  if (name == "hip_ext_th") {
    float v = valueStr.toFloat();
    if (v < -40.0f || v > 20.0f) {
      hostPrintln("ERROR: hip_ext_th out of range (-40.0~20.0)");
      return false;
    }
    torqueParams.hip_ext_th = v;
    hostPrintf(">>> Set hip_ext_th=%.2f deg\n", torqueParams.hip_ext_th);
    return true;
  }
  if (name == "pushoff_max_ms") {
    uint32_t v = (uint32_t)valueStr.toInt();
    if (v < 50 || v > 1000) {
      hostPrintln("ERROR: pushoff_max_ms out of range (50~1000)");
      return false;
    }
    torqueParams.pushoff_max_ms = v;
    hostPrintf(">>> Set pushoff_max_ms=%lu ms\n", (unsigned long)torqueParams.pushoff_max_ms);
    return true;
  }
  if (name == "ankle_pf_target_deg") {
    float v = valueStr.toFloat();
    if (v < -10.0f || v > 30.0f) {
      hostPrintln("ERROR: ankle_pf_target_deg out of range (-10.0~30.0)");
      return false;
    }
    if (v >= torqueParams.ankle_df_th) {
      hostPrintln("ERROR: ankle_pf_target_deg must be less than ankle_df_th");
      return false;
    }
    torqueParams.ankle_pf_target_deg = v;
    hostPrintf(">>> Set ankle_pf_target_deg=%.2f deg\n", torqueParams.ankle_pf_target_deg);
    return true;
  }
  if (name == "iq_pf_max") {
    int v = valueStr.toInt();
    if (v < 1 || v > 2000) {
      hostPrintln("ERROR: iq_pf_max out of range (1~2000)");
      return false;
    }
    if (torqueParams.iq_pf_floor > v) {
      hostPrintln("ERROR: iq_pf_max must be >= iq_pf_floor");
      return false;
    }
    torqueParams.iq_pf_max = (int16_t)v;
    hostPrintf(">>> Set iq_pf_max=%d\n", (int)torqueParams.iq_pf_max);
    return true;
  }
  if (name == "iq_pf_floor") {
    int v = valueStr.toInt();
    if (v < 0 || v > 2000) {
      hostPrintln("ERROR: iq_pf_floor out of range (0~2000)");
      return false;
    }
    if (v > torqueParams.iq_pf_max) {
      hostPrintln("ERROR: iq_pf_floor must be <= iq_pf_max");
      return false;
    }
    torqueParams.iq_pf_floor = (int16_t)v;
    hostPrintf(">>> Set iq_pf_floor=%d\n", (int)torqueParams.iq_pf_floor);
    return true;
  }
  if (name == "diq_up_pf") {
    int v = valueStr.toInt();
    if (v < 1 || v > 1000) {
      hostPrintln("ERROR: diq_up_pf out of range (1~1000)");
      return false;
    }
    torqueParams.diq_up_pf = (int16_t)v;
    hostPrintf(">>> Set diq_up_pf=%d\n", (int)torqueParams.diq_up_pf);
    return true;
  }
  if (name == "ankleBypassSafety") {
    int v = valueStr.toInt();
    torqueParams.ankleBypassSafety = (v != 0);
    // 切换 bypass 时同步重置安全状态，避免遗留 compliant/cooldown 影响
    ankleSafety.compliant    = false;
    ankleSafety.in_cooldown  = false;
    ankleSafety.iq_cmd_prev  = 0;
    hostPrintf(">>> Set ankleBypassSafety=%d (%s)\n",
               torqueParams.ankleBypassSafety ? 1 : 0,
               torqueParams.ankleBypassSafety ? "旁路/测试模式" : "安全管线模式");
    return true;
  }

  hostPrintln("ERROR: Unknown A1 param name");
  hostPrintln("       Supported: ankle_df_th, hip_ext_th, pushoff_max_ms, ankle_pf_target_deg, iq_pf_max, iq_pf_floor, diq_up_pf, ankleBypassSafety");
  return false;
}

bool getA1Param(const String& name) {
  if (name == "ankle_df_th") {
    hostPrintf(">>> ankle_df_th=%.2f\n", torqueParams.ankle_df_th);
    return true;
  }
  if (name == "hip_ext_th") {
    hostPrintf(">>> hip_ext_th=%.2f\n", torqueParams.hip_ext_th);
    return true;
  }
  if (name == "pushoff_max_ms") {
    hostPrintf(">>> pushoff_max_ms=%lu\n", (unsigned long)torqueParams.pushoff_max_ms);
    return true;
  }
  if (name == "ankle_pf_target_deg") {
    hostPrintf(">>> ankle_pf_target_deg=%.2f\n", torqueParams.ankle_pf_target_deg);
    return true;
  }
  if (name == "iq_pf_max") {
    hostPrintf(">>> iq_pf_max=%d\n", (int)torqueParams.iq_pf_max);
    return true;
  }
  if (name == "iq_pf_floor") {
    hostPrintf(">>> iq_pf_floor=%d\n", (int)torqueParams.iq_pf_floor);
    return true;
  }
  if (name == "diq_up_pf") {
    hostPrintf(">>> diq_up_pf=%d\n", (int)torqueParams.diq_up_pf);
    return true;
  }
  if (name == "ankleBypassSafety") {
    hostPrintf(">>> ankleBypassSafety=%d\n", torqueParams.ankleBypassSafety ? 1 : 0);
    return true;
  }
  hostPrintln("ERROR: Unknown A1 param name");
  return false;
}

// ============================================================================
// 串口命令处理
// ============================================================================

void processSerialCommand() {
  Stream* lineSrc = nullptr;
  if (Serial.available()) {
    lineSrc = &Serial;
  } else if (BT_SERIAL.available()) {
    lineSrc = &BT_SERIAL;
  } else {
    return;
  }

  String line = lineSrc->readStringUntil('\n');
  line.trim();

  if (line.length() == 0) return;

  CmdReplyScope replyScope(lineSrc);

  // 如果正在接收步态数据，将数据传递给处理函数
  if (gaitDataReceive.receiving) {
    processReceivedGaitData(line);
    return;
  }

  String cmd = line;
  cmd.toLowerCase();

  hostPrintf("> Command: %s\n", cmd.c_str());
  
  // 使能命令
  if (cmd == "e1" || cmd == "enable1") {
    enableMotor(hipMotor);
  } else if (cmd == "e2" || cmd == "enable2") {
    enableMotor(ankleMotor);
  } else if (cmd == "e" || cmd == "enable") {
    enableMotor(hipMotor);
    enableMotor(ankleMotor);
  }
  // 掉电命令
  else if (cmd == "d1" || cmd == "disable1") {
    disableMotor(hipMotor);
  } else if (cmd == "d2" || cmd == "disable2") {
    disableMotor(ankleMotor);
  } else if (cmd == "d" || cmd == "disable") {
    disableMotor(hipMotor);
    disableMotor(ankleMotor);
  }
  // 停止命令
  else if (cmd == "s1") {
    stopMotor(hipMotor);
  } else if (cmd == "s2") {
    stopMotor(ankleMotor);
  }
  // 清除错误
  else if (cmd == "ce1" || cmd == "clearerror1") {
    clearMotorError(hipMotor);
  } else if (cmd == "ce2" || cmd == "clearerror2") {
    clearMotorError(ankleMotor);
  } else if (cmd == "ce" || cmd == "clearerror") {
    clearMotorError(hipMotor);
    clearMotorError(ankleMotor);
  }
  // 读取角度
  else if (cmd == "r1" || cmd == "read1") {
    requestMotorAngle(hipMotor);
  } else if (cmd == "r2" || cmd == "read2") {
    requestMotorAngle(ankleMotor);
  } else if (cmd == "r" || cmd == "read") {
    requestMotorAngle(hipMotor);
    requestMotorAngle(ankleMotor);
  }
  // 显示状态
  else if (cmd == "s" || cmd == "status") {
    // 1. 主动请求读取两个电机当前角度和状态
    requestMotorAngle(hipMotor);
    requestMotorAngle(ankleMotor);

    // 等待最多100ms以确保收到CAN回复（每10ms检查一次）
    const uint32_t statusTimeoutMs = 100;
    uint32_t t_start = millis();
    bool hipOk = false, ankleOk = false;
    while (millis() - t_start < statusTimeoutMs) {
      // 处理CAN消息队列
      CAN_message_t inMsg;
      while (can1.read(inMsg)) {
        handleCanMessage(inMsg);
      }
      // 检查数据新鲜性
      uint32_t now = millis();
      hipOk = (hipStatus.lastUpdateMs > 0) && ((now - hipStatus.lastUpdateMs) < 300);
      ankleOk = (ankleStatus.lastUpdateMs > 0) && ((now - ankleStatus.lastUpdateMs) < 300);
      if (hipOk && ankleOk) break;
      delay(10);
    }

    // 再次确认
    uint32_t now = millis();
    hipOk = (hipStatus.lastUpdateMs > 0) && ((now - hipStatus.lastUpdateMs) < 300);
    ankleOk = (ankleStatus.lastUpdateMs > 0) && ((now - ankleStatus.lastUpdateMs) < 300);

    if (!hipOk || !ankleOk) {
      hostPrintln("\n[ERROR] Failed to get latest motor status! (try again)");
      if (!hipOk) hostPrintln(" - Hip motor data is NOT up to date.");
      if (!ankleOk) hostPrintln(" - Ankle motor data is NOT up to date.");
      hostPrintln("Aborted status display.");
      return;
    }

    // 数据新鲜，输出信息
    hostPrintln("\n=== Motor Status ===");
    hostPrintf("Hip:   angle=%.2f deg (logical, raw=%lld units), speed=%d dps, enabled=%d, state=0x%02X\n",
                  getHipDeg(), static_cast<long long>(getHipRawUnits()),
                  hipStatus.speed, hipStatus.enabled, hipStatus.motorState);
    hostPrintf("Ankle: angle=%.2f deg (logical, raw=%lld units), speed=%d dps, enabled=%d, state=0x%02X\n",
                  getAnkleDeg(), static_cast<long long>(getAnkleRawUnits()),
                  ankleStatus.speed, ankleStatus.enabled, ankleStatus.motorState);

    // 显示髋关节信号预处理状态
    if (hipProcessor.initialized) {
      hostPrintln("\n=== Hip Signal Processing ===");
      hostPrintf("Raw angle:     %.2f deg (logical)\n", getHipDeg());
      hostPrintf("Filtered:      %.2f deg\n", hipProcessor.hip_f);
      hostPrintf("Velocity:      %.2f deg/s\n", hipProcessor.hip_vel);
      hostPrintf("Vel filtered:  %.2f deg/s\n", hipProcessor.hip_vel_f);
    } else {
      hostPrintln("\n=== Hip Signal Processing ===");
      hostPrintln("Not initialized (need hip angle data)");
    }

    // 显示髋关节标定状态
    hostPrintln("\n=== Hip Calibration ===");
    if (hip_reference_set) {
      hostPrintf("Calibrated: YES (offset=%lld units)\n", 
                   static_cast<long long>(hip_reference_offset));
    } else {
      hostPrintln("Calibrated: NO (use 'hz' command to calibrate)");
    }

    // 显示踝关节标定状态
    hostPrintln("\n=== Ankle Calibration ===");
    if (ankle_zero_calibrated) {
      hostPrintf("Calibrated: YES (offset=%lld units)\n", 
                   static_cast<long long>(ankle_zero_offset));
    } else {
      hostPrintln("Calibrated: NO (use 'az' command to calibrate)");
    }
  }
  // 步态数据采集命令
  else if (cmd == "gc" || cmd == "gaitcollect" || cmd == "gaitstart") {
    startGaitCollection(20);  // 默认20ms间隔
  } else if (cmd.startsWith("gc ")) {
    // gc 20 表示20ms间隔采集
    uint32_t interval = cmd.substring(3).toInt();
    if (interval >= 10 && interval <= 1000) {
      startGaitCollection(interval);
    } else {
      hostPrintln("ERROR: Interval must be 10-1000 ms");
    }
  } else if (cmd == "gcs" || cmd == "gaitstop") {
    stopGaitCollection();
  }
  // 蓝牙遥测开关：bt / bt on / bt off
  else if (cmd == "bt" || cmd == "bt status") {
    hostPrintf(">>> Bluetooth telemetry: %s\n",
               useBluetoothTelemetry ? "ON (compact payload: t,h,a)" : "OFF (USB full payload)");
  }
  else if (cmd == "bt on" || cmd == "bton") {
    useBluetoothTelemetry = true;
    hostPrintln(">>> Bluetooth telemetry ENABLED (compact payload: t,h,a)");
  }
  else if (cmd == "bt off" || cmd == "btoff") {
    useBluetoothTelemetry = false;
    hostPrintln(">>> Bluetooth telemetry DISABLED (USB full payload)");
  }
  // 步态轨迹播放命令：
  //   - gp <freq>            ：只指定周期频率，最大速度由程序根据轨迹自动计算，保证尽量走完整幅度
  //   - gp <freq> <speed>    ：指定周期频率和最大速度（高级用法）
  else if (cmd.startsWith("gp ")) {
    int space1 = cmd.indexOf(' ');
    if (space1 < 0) {
      hostPrintln("ERROR: Usage: gp <freq> [speed]");
    } else {
      int space2 = cmd.indexOf(' ', space1 + 1);
      // 只有频率参数：gp <freq>，速度自动计算
      if (space2 < 0) {
        float freq = cmd.substring(space1 + 1).toFloat();
        if (freq > 0 && freq <= 5.0f) {
          // 传入 maxSpeedDps = 0，触发自动计算模式
          startGaitPlayback(freq, 0.0f);
        } else {
          hostPrintln("ERROR: Frequency must be 0-5 Hz");
        }
      } else {
        // 兼容旧用法：gp <freq> <speed>
        float freq = cmd.substring(space1 + 1, space2).toFloat();
        float speed = cmd.substring(space2 + 1).toFloat();
        if (freq > 0 && freq <= 5.0f && speed > 0 && speed <= 1000.0f) {
          startGaitPlayback(freq, speed);
        } else {
          hostPrintln("ERROR: Frequency must be 0-5 Hz, speed must be 0-1000 dps");
        }
      }
    }
  } else if (cmd == "gps" || cmd == "gaitplaystop") {
    stopGaitPlayback();
  }
  // 加载步态数据命令：loadgait 开始接收JSON格式的步态数据
  else if (cmd == "loadgait" || cmd == "loadtrajectory") {
    startReceivingGaitData();
  }
  // 踝关节零点标定命令：az（ankle zero）
  // 在用户站立自然中立位时执行，将当前踝关节角度设为0度
  else if (cmd == "az" || cmd == "anklezero") {
    // 先读取当前踝关节角度
    requestMotorAngle(ankleMotor);
    delay(50);  // 等待回复
    {
      CAN_message_t inMsg;
      while (can1.read(inMsg)) {
        handleCanMessage(inMsg);
      }
    }
    
    // 如果成功读取到角度，保存为零点偏移
    if (ankleStatus.lastUpdateMs > 0 && 
        (millis() - ankleStatus.lastUpdateMs) < 200) {  // 确保数据是新鲜的
      ankle_zero_offset = ankleStatus.multiTurnAngle;
      ankle_zero_calibrated = true;
      hostPrintln(">>> Ankle zero calibration SUCCESS");
      hostPrintf(">>> Zero offset: %lld units (%.2f deg)\n", 
                   static_cast<long long>(ankle_zero_offset),
                   unitsToAngleDeg(ankleMotor, ankle_zero_offset));
      hostPrintln(">>> Ankle angle will now be calculated relative to this zero position");
      hostPrintln(">>> 0 deg = foot at 90° to shank (neutral position)");
    } else {
      hostPrintln("ERROR: Failed to read ankle angle. Please try again.");
    }
  }
  // 髋关节零点标定命令：hz（hip zero）
  // 在用户站立自然中立位时执行，将当前髋关节角度设为0度（参考姿态）
  else if (cmd == "hz" || cmd == "hipzero") {
    // 先读取当前髋关节角度
    requestMotorAngle(hipMotor);
    delay(50);  // 等待回复
    {
      CAN_message_t inMsg;
      while (can1.read(inMsg)) {
        handleCanMessage(inMsg);
      }
    }
    
    // 如果成功读取到角度，保存为参考姿态偏移
    if (hipStatus.lastUpdateMs > 0 && 
        (millis() - hipStatus.lastUpdateMs) < 200) {  // 确保数据是新鲜的
      hip_reference_offset = hipStatus.raw_units;
      hip_reference_set = true;
      hostPrintln(">>> Hip zero calibration SUCCESS");
      hostPrintf(">>> Reference offset: %lld units (%.2f deg)\n", 
                   static_cast<long long>(hip_reference_offset),
                   unitsToAngleDeg(hipMotor, hip_reference_offset));
      hostPrintln(">>> Hip angle will now be calculated relative to this reference position");
      hostPrintln(">>> 0 deg = reference posture");
    } else {
      hostPrintln("ERROR: Failed to read hip angle. Please try again.");
    }
  }
  // 自适应阈值调试命令：th（threshold）
  else if (cmd == "th" || cmd == "threshold") {
    if (adaptiveThreshold.initialized) {
      hostPrintln(">>> Adaptive Threshold Status:");
      hostPrintf(">>>   Window: %d/%d samples\n", adaptiveThreshold.windowCount, HIP_WINDOW_SIZE);
      hostPrintf(">>>   Hip Mean: %.2f deg\n", adaptiveThreshold.hip_mean);
      hostPrintf(">>>   Hip Amplitude: %.2f deg\n", adaptiveThreshold.hip_amp);
      hostPrintf(">>>   A_up: %.2f deg\n", adaptiveThreshold.A_up);
      hostPrintf(">>>   A_dn: %.2f deg\n", adaptiveThreshold.A_dn);
      hostPrintf(">>>   V_up: %.2f deg/s\n", adaptiveThreshold.V_up);
      hostPrintf(">>>   V_dn: %.2f deg/s\n", adaptiveThreshold.V_dn);
      hostPrintf(">>>   T_hold: %lu ms\n", T_HOLD_MS);
      if (hipProcessor.initialized) {
        hostPrintf(">>>   Current hip_f: %.2f deg\n", hipProcessor.hip_f);
        hostPrintf(">>>   Current hip_vel_f: %.2f deg/s\n", hipProcessor.hip_vel_f);
        hostPrintf(">>>   Swing condition: hip_vel_f > V_up && hip_f > hip_mean + A_up\n");
        hostPrintf(">>>   Stance condition: hip_vel_f < V_dn && hip_f < hip_mean - A_dn\n");
      }
    } else {
      hostPrintln(">>> Adaptive Threshold: NOT INITIALIZED");
      hostPrintln(">>> Start gait collection (gc) to initialize threshold calculation");
    }
  }
  // 步态相位调试命令：phase
  else if (cmd == "phase" || cmd == "gaitphase") {
    if (gaitPhaseDetector.initialized) {
      hostPrintln(">>> Gait Phase Detection Status:");
      hostPrintf(">>>   Current Phase: %s\n", 
                   gaitPhaseDetector.currentPhase == PHASE_SWING ? "SWING" : "STANCE");
      hostPrintf(">>>   Phase Duration: %lu ms (%.2f s)\n", 
                   getCurrentPhaseDurationMs(),
                   getCurrentPhaseDurationMs() / 1000.0f);
      hostPrintf(">>>   Condition Hold Time: %lu ms\n", gaitPhaseDetector.conditionHoldMs);
      if (hipProcessor.initialized && adaptiveThreshold.initialized) {
        float hip_f = hipProcessor.hip_f;
        float hip_vel_f = hipProcessor.hip_vel_f;
        float hip_mean = adaptiveThreshold.hip_mean;
        float A_up = adaptiveThreshold.A_up;
        float A_dn = adaptiveThreshold.A_dn;
        float V_up = adaptiveThreshold.V_up;
        float V_dn = adaptiveThreshold.V_dn;
        
        // 修正：swing和stance的条件判断反了，需要互换
        bool swingConditionMet = (hip_vel_f < V_dn) && (hip_f < hip_mean - A_dn);  // 速度小且角度小 -> SWING
        bool stanceConditionMet = (hip_vel_f > V_up) && (hip_f > hip_mean + A_up);  // 速度大且角度大 -> STANCE
        
        hostPrintf(">>>   Current hip_f: %.2f deg\n", hip_f);
        hostPrintf(">>>   Current hip_vel_f: %.2f deg/s\n", hip_vel_f);
        hostPrintf(">>>   hip_mean: %.2f deg\n", hip_mean);
        hostPrintf(">>>   Swing condition (vel<%.1f && angle<%.2f): %s\n", 
                     V_dn, hip_mean - A_dn, swingConditionMet ? "YES" : "NO");
        hostPrintf(">>>   Stance condition (vel>%.1f && angle>%.2f): %s\n", 
                     V_up, hip_mean + A_up, stanceConditionMet ? "YES" : "NO");
      }
      // 显示摆动进度信息
      if (swingProgress.initialized) {
        hostPrintln(">>> Swing Progress:");
        hostPrintf(">>>   Ts (avg period): %.3f s\n", swingProgress.Ts);
        hostPrintf(">>>   t_swing (current): %.3f s\n", swingProgress.t_swing);
        hostPrintf(">>>   s (progress): %.3f (%.1f%%)\n", 
                     swingProgress.swing_progress,
                     swingProgress.swing_progress * 100.0f);
      }
    } else {
      hostPrintln(">>> Gait Phase Detector: NOT INITIALIZED");
      hostPrintln(">>> Start gait collection (gc) to initialize phase detection");
    }
  }
  // 4相步态识别调试命令：phase4
  else if (cmd == "phase4" || cmd == "ph4") {
    if (phase4Det.initialized) {
      const char* phaseNames[4] = {"LOADING", "MID_STANCE", "PUSH_OFF", "SWING"};
      hostPrintln(">>> 4-Phase Gait Detection Status:");
      hostPrintf(">>>   Current Phase: %s (%d)\n",
                   phaseNames[(int)phase4Det.currentPhase], (int)phase4Det.currentPhase);
      hostPrintf(">>>   Phase Progress: %.3f (%.1f%%)\n",
                   phase4Det.phaseProgress, phase4Det.phaseProgress * 100.0f);
      hostPrintf(">>>   Profile Output (Gaussian): %.3f\n", phase4Det.profileOutput);
      hostPrintf(">>>   Phase Duration: %lu ms\n", millis() - phase4Det.phaseStartMs);
      hostPrintf(">>>   Degraded Mode: %s\n", phase4Det.degraded ? "YES" : "NO");
      if (phase4Det.degraded) {
        hostPrintf(">>>   Degraded for: %lu ms (recovers after %lu ms)\n",
                     millis() - phase4Det.degradedStartMs,
                     PHASE4_RECOVERY_MS);
      }
      hostPrintf(">>>   Transition Count (3s window): %d / %d\n",
                   phase4Det.transitionCount, PHASE4_UNSTABLE_TRANS_THRESH);
      hostPrintln(">>>   Thresholds:");
      hostPrintf(">>>     P1→P2 stance_pct: %.2f (%.0f%%)\n",
                   PHASE4_P1P2_STANCE_PCT, PHASE4_P1P2_STANCE_PCT * 100.0f);
      hostPrintf(">>>     P2→P3 stance_pct: %.2f (%.0f%%)\n",
                   PHASE4_P2P3_STANCE_PCT, PHASE4_P2P3_STANCE_PCT * 100.0f);
      hostPrintf(">>>     Debounce: %lu ms\n", PHASE4_DEBOUNCE_MS);
      hostPrintln(">>>   Phase Profiles (amp/center/width/ramp):");
      const char* pnames[4] = {"LOADING   ", "MID_STANCE", "PUSH_OFF  ", "SWING     "};
      for (int i = 0; i < 4; i++) {
        hostPrintf(">>>     %s: amp=%.2f ctr=%.2f wid=%.2f ramp=%.3f\n",
                     pnames[i], phaseProfiles[i].amp, phaseProfiles[i].center,
                     phaseProfiles[i].width, phaseProfiles[i].ramp_limit);
      }
      // 显示底层2相状态
      if (gaitPhaseDetector.initialized) {
        hostPrintf(">>>   Base Phase (2-phase): %s, stance_pct=%.3f\n",
                     gaitPhaseDetector.currentPhase == PHASE_SWING ? "SWING" : "STANCE",
                     getStancePct(gaitPhaseDetector.currentPhase, millis()));
      }
    } else {
      hostPrintln(">>> 4-Phase Detector: NOT INITIALIZED");
      hostPrintln(">>> Enable control loop (ctrlon) to initialize phase detection");
    }
  }
  else if (cmd == "ph4 on" || cmd == "phase4 on") {
    phase4Monitor.enabled = true;
    phase4Monitor.lastSendMs = 0;
    phase4Monitor.sendIntervalMs = 100;
    hostPrintln(">>> PH4 realtime monitor ENABLED");
    hostPrintln(">>> Output format: JSON line, key `ph4rt` marks realtime phase4 stream");
    hostPrintf(">>> Interval: %lu ms (%.1f Hz)\n",
                  phase4Monitor.sendIntervalMs, 1000.0f / phase4Monitor.sendIntervalMs);
  }
  else if (cmd == "ph4 off" || cmd == "phase4 off") {
    phase4Monitor.enabled = false;
    hostPrintln(">>> PH4 realtime monitor DISABLED");
  }
  else if (cmd.startsWith("ph4 ") || cmd.startsWith("phase4 ")) {
    int spaceIdx = cmd.indexOf(' ');
    String arg = cmd.substring(spaceIdx + 1);
    arg.trim();
    if (arg == "on") {
      phase4Monitor.enabled = true;
      phase4Monitor.lastSendMs = 0;
      phase4Monitor.sendIntervalMs = 100;
      hostPrintf(">>> PH4 realtime monitor ENABLED (%lu ms)\n", phase4Monitor.sendIntervalMs);
    } else if (arg == "off") {
      phase4Monitor.enabled = false;
      hostPrintln(">>> PH4 realtime monitor DISABLED");
    } else {
      uint32_t interval = arg.toInt();
      if (interval >= 20 && interval <= 2000) {
        phase4Monitor.enabled = true;
        phase4Monitor.lastSendMs = 0;
        phase4Monitor.sendIntervalMs = interval;
        hostPrintf(">>> PH4 realtime monitor ENABLED (%lu ms, %.1f Hz)\n",
                      interval, 1000.0f / interval);
      } else {
        hostPrintln("ERROR: Usage: ph4 | ph4 on | ph4 off | ph4 <interval_ms>");
        hostPrintln("       Interval range: 20-2000 ms");
      }
    }
  }
  // 摆动进度调试命令：swing
  else if (cmd == "swing" || cmd == "swingprogress") {
    if (swingProgress.initialized) {
      hostPrintln(">>> Swing Progress Status:");
      hostPrintf(">>>   Ts (Average Period): %.3f s\n", swingProgress.Ts);
      hostPrintf(">>>   t_swing (Current Duration): %.3f s\n", swingProgress.t_swing);
      hostPrintf(">>>   s (Progress): %.3f (%.1f%%)\n", 
                   swingProgress.swing_progress,
                   swingProgress.swing_progress * 100.0f);
      if (gaitPhaseDetector.initialized) {
        hostPrintf(">>>   Current Phase: %s\n", 
                     gaitPhaseDetector.currentPhase == PHASE_SWING ? "SWING" : "STANCE");
        hostPrintf(">>>   Phase Duration: %lu ms\n", getCurrentPhaseDurationMs());
      }
    } else {
      hostPrintln(">>> Swing Progress: NOT INITIALIZED");
      hostPrintln(">>> Start gait collection (gc) to initialize swing progress calculation");
    }
  }
  // 踝背屈辅助策略调试命令：assist
  else if (cmd == "assist" || cmd == "ankleassist") {
    if (ankleAssist.initialized) {
      hostPrintln(">>> Ankle Dorsiflexion Assist Strategy Status:");
      hostPrintf(">>>   Enabled: %s\n", ankleAssist.enabled ? "YES" : "NO");
      hostPrintf(">>>   Parameters:\n");
      hostPrintf(">>>     θ_low:  %.2f deg\n", ANKLE_THETA_LOW);
      hostPrintf(">>>     θ_high: %.2f deg (configurable, use 'dorsiflex <angle>' to set)\n", ankleAssist.dorsiflexion_target);
      hostPrintf(">>>     θ_min:  %.2f deg (safety limit)\n", ANKLE_THETA_MIN);
      hostPrintf(">>>     θ_max:  %.2f deg (safety limit)\n", ANKLE_THETA_MAX);
      hostPrintf(">>>   Current Values:\n");
      hostPrintf(">>>     Current Ankle Angle: %.2f deg (logical)\n", getAnkleDeg());
      hostPrintf(">>>     Target Angle (S-curve): %.2f deg\n", ankleAssist.theta_target);
      hostPrintf(">>>     Reference Angle: %.2f deg\n", ankleAssist.theta_ref);
      hostPrintf(">>>     Assist Factor: %.3f (%.1f%%)\n", 
                   ankleAssist.assist_factor,
                   ankleAssist.assist_factor * 100.0f);
      if (gaitPhaseDetector.initialized) {
        GaitPhase currentPhase = getCurrentGaitPhase();
        hostPrintf(">>>   Current Phase: %s\n", 
                     currentPhase == PHASE_SWING ? "SWING (assist active)" : "STANCE (assist inactive)");
        if (currentPhase == PHASE_SWING && swingProgress.initialized) {
          float s = getSwingProgress();
          hostPrintf(">>>   Swing Progress: %.3f (%.1f%%)\n", s, s * 100.0f);
        }
      }
    } else {
      hostPrintln(">>> Ankle Assist Strategy: NOT INITIALIZED");
      hostPrintln(">>> Start gait collection (gc) to initialize assist strategy");
    }
  }
  // 启用/禁用踝背屈辅助：assiston / assistoff
  else if (cmd == "assiston") {
    setAnkleAssistEnabled(true);
    hostPrintln(">>> Ankle dorsiflexion assist ENABLED");
  }
  else if (cmd == "assistoff") {
    setAnkleAssistEnabled(false);
    hostPrintln(">>> Ankle dorsiflexion assist DISABLED");
  }
  // 髋关节力矩模式控制（串口调试）
  else if (cmd.startsWith("hiptorque")) {
    // 支持："hiptorque on <iq>", "hiptorque off", "hiptorque"(显示状态)
    if (cmd == "hiptorque") {
      hostPrintf(">>> Hip torque mode: %s, target iq=%d\n", hipTorqueMode ? "ON" : "OFF", hipIqTarget);
    } else if (cmd.startsWith("hiptorque on")) {
      int space = cmd.indexOf(' ', 12);
      if (space > 0) {
        int16_t iq = cmd.substring(space + 1).toInt();
        hipIqTarget = iq;
      }
      hipTorqueMode = true;
      hostPrintf(">>> Hip torque MODE ENABLED, target iq=%d\n", hipIqTarget);
    } else if (cmd == "hiptorque off" || cmd == "hiptorque disable") {
      hipTorqueMode = false;
      hostPrintln(">>> Hip torque MODE DISABLED");
    }
  }
  // 踝关节力矩模式控制（串口调试）
  else if (cmd.startsWith("ak") || cmd.startsWith("ankletorque")) {
    // 支持：
    //   ak
    //   ak on <iq>
    //   ak off
    //   ankletorque
    //   ankletorque on <iq>
    //   ankletorque off
    if (cmd == "ak" || cmd == "ankletorque") {
      hostPrintf(">>> Ankle torque mode: %s, target iq=%d\n",
                    ankleTorqueMode ? "ON" : "OFF", ankleIqTargetManual);
      queryAndPrintAnkleIqTs();
    } else if (cmd == "ak read" || cmd == "ankletorque read") {
      queryAndPrintAnkleIqTs();
    } else if (cmd.startsWith("ak on ") || cmd.startsWith("ankletorque on ")) {
      int lastSpace = cmd.lastIndexOf(' ');
      if (lastSpace > 0 && lastSpace < (int)cmd.length() - 1) {
        int16_t iq = cmd.substring(lastSpace + 1).toInt();
        ankleIqTargetManual = iq;
        ankleTorqueMode = true;
        hostPrintf(">>> Ankle torque MODE ENABLED, target iq=%d\n", ankleIqTargetManual);
        queryAndPrintAnkleIqTs();
      } else {
        hostPrintln("ERROR: Usage: ak on <iq> / ankletorque on <iq>");
      }
    } else if (cmd == "ak off" || cmd == "ak disable" ||
               cmd == "ankletorque off" || cmd == "ankletorque disable") {
      ankleTorqueMode = false;
      hostPrintln(">>> Ankle torque MODE DISABLED");
    } else {
      hostPrintln("ERROR: Usage: ak | ak on <iq> | ak off | ak read");
      hostPrintln("       or: ankletorque | ankletorque on <iq> | ankletorque off | ankletorque read");
    }
  }
  // 顺从控制调试命令：compliance
  else if (cmd == "compliance" || cmd == "comp") {
    if (complianceCtrl.initialized) {
      hostPrintln(">>> Compliance Control Status:");
      const char* stateNames[] = {"NORMAL", "COMPLIANT", "HOLD", "FAULT_SAFE"};
      hostPrintf(">>>   Current State: %s\n", stateNames[complianceCtrl.currentState]);
      hostPrintf(">>>   State Duration: %lu ms\n", getComplianceStateDuration());
      hostPrintf(">>>   Speed Factor: %.2f (%.0f%%)\n", 
                   complianceCtrl.maxSpeedFactor,
                   complianceCtrl.maxSpeedFactor * 100.0f);
      hostPrintf(">>>   Parameters:\n");
      hostPrintf(">>>     I1 (轻度阻力): %d mA\n", COMPLIANCE_I1);
      hostPrintf(">>>     I2 (重度阻力): %d mA\n", COMPLIANCE_I2);
      hostPrintf(">>>     E1 (位置误差1): %.1f deg\n", COMPLIANCE_E1);
      hostPrintf(">>>     E2 (位置误差2): %.1f deg\n", COMPLIANCE_E2);
      hostPrintf(">>>     T_resist: %lu ms\n", COMPLIANCE_T_RESIST);
      hostPrintf(">>>   Current Values:\n");
      hostPrintf(">>>     Ankle Angle: %.2f deg (logical)\n", getAnkleDeg());
      hostPrintf(">>>     Reference Angle: %.2f deg\n", getAnkleReferenceAngle());
      float posError = fabsf(getAnkleReferenceAngle() - getAnkleDeg());
      hostPrintf(">>>     Position Error: %.2f deg\n", posError);
      hostPrintf(">>>     Current (iq): %d mA\n", ankleStatus.iq);
      float iq_abs = fabsf((float)ankleStatus.iq);
      hostPrintf(">>>     |iq|: %.1f mA\n", iq_abs);
      hostPrintf(">>>     Temperature: %d ℃\n", ankleStatus.temperature);
      bool commOk = (millis() - ankleStatus.lastUpdateMs) < COMM_TIMEOUT_MS;
      hostPrintf(">>>     Communication: %s\n", commOk ? "OK" : "TIMEOUT");
      if (complianceCtrl.currentState == STATE_HOLD) {
        hostPrintf(">>>     Hold Position: %.2f deg\n", complianceCtrl.positionHold);
      }
      if (complianceCtrl.lowResistanceStartMs > 0) {
        uint32_t lowResistDuration = millis() - complianceCtrl.lowResistanceStartMs;
        hostPrintf(">>>     Low Resistance Duration: %lu ms (need %lu ms to exit)\n", 
                     lowResistDuration, COMPLIANCE_T_RESIST);
      }
    } else {
      hostPrintln(">>> Compliance Control: NOT INITIALIZED");
      hostPrintln(">>> Start gait collection (gc) to initialize compliance control");
    }
  }
  // 重置故障状态：resetfault
  else if (cmd == "resetfault" || cmd == "reset") {
    if (complianceCtrl.currentState == STATE_FAULT_SAFE) {
      resetComplianceFault();
      hostPrintln(">>> Fault state RESET to NORMAL");
    } else {
      hostPrintln(">>> Current state is not FAULT_SAFE, no reset needed");
      hostPrintf(">>> Current state: %d\n", complianceCtrl.currentState);
    }
  }
  // 启用/禁用控制循环：ctrlon / ctrloff
  else if (cmd == "ctrlon" || cmd == "controlon") {
    setControlLoopEnabled(true);
  }
  else if (cmd == "ctrloff" || cmd == "controloff") {
    setControlLoopEnabled(false);
  }
  // A1 参数设置：set <name> <value>
  else if (cmd.startsWith("set ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (firstSpace < 0 || secondSpace < 0) {
      hostPrintln("ERROR: Usage: set <name> <value>");
      hostPrintln("       Example: set ankle_df_th 12");
      hostPrintln("       Names: ankle_df_th, hip_ext_th, pushoff_max_ms, ankle_pf_target_deg, iq_pf_max, iq_pf_floor, diq_up_pf, ankleBypassSafety");
    } else {
      String name = cmd.substring(firstSpace + 1, secondSpace);
      String value = cmd.substring(secondSpace + 1);
      name.trim();
      value.trim();
      if (name.length() == 0 || value.length() == 0) {
        hostPrintln("ERROR: Usage: set <name> <value>");
      } else {
        if (setA1Param(name, value)) {
          saveA1ParamsToEeprom();
        }
      }
    }
  }
  // A1 参数读取：get <name> / params
  else if (cmd.startsWith("get ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: get <name>");
    } else {
      String name = cmd.substring(spaceIdx + 1);
      name.trim();
      if (name.length() == 0) {
        hostPrintln("ERROR: Usage: get <name>");
      } else {
        getA1Param(name);
      }
    }
  }
  else if (cmd == "params") {
    printA1Params();
  }
  // 设置踝关节电机速度参数：speed <value> 或 motorspeed <value>
  // value 为协议单位（电机轴速度 dps），范围建议 100-10000
  else if (cmd.startsWith("speed ") || cmd.startsWith("motorspeed ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: speed <value> (e.g., speed 5000)");
      hostPrintln("       Value range: 100-10000 (protocol units, motor axis speed dps)");
    } else {
      uint16_t speed = cmd.substring(spaceIdx + 1).toInt();
      if (speed >= 100 && speed <= 10000) {
        controlLoop.motorSpeed = speed;
        hostPrintf(">>> Ankle motor speed set to: %u (protocol units, motor axis speed dps)\n", speed);
      } else {
        hostPrintln("ERROR: Speed must be 100-10000 (protocol units)");
      }
    }
  }
  // 查询当前踝关节电机速度参数
  else if (cmd == "speed" || cmd == "motorspeed") {
    hostPrintf(">>> Current ankle motor speed: %u (protocol units, motor axis speed dps)\n", 
                  controlLoop.motorSpeed);
  }
  // 设置踝关节背屈目标角度：dorsiflex <angle> 或 df <angle>
  // angle 为背屈目标角度（度），范围建议 0-40
  else if (cmd.startsWith("dorsiflex ") || cmd.startsWith("df ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: dorsiflex <angle> or df <angle> (e.g., dorsiflex 8.0)");
      hostPrintln("       Angle range: 0-40 degrees");
    } else {
      float angle = cmd.substring(spaceIdx + 1).toFloat();
      if (angle >= 0.0f && angle <= 40.0f) {
        ankleAssist.dorsiflexion_target = angle;
        hostPrintf(">>> Ankle dorsiflexion target set to: %.2f degrees\n", angle);
      } else {
        hostPrintln("ERROR: Angle must be 0-40 degrees");
      }
    }
  }
  // 查询当前踝关节背屈目标角度
  else if (cmd == "dorsiflex" || cmd == "df") {
    hostPrintf(">>> Current ankle dorsiflexion target: %.2f degrees\n", 
                  ankleAssist.dorsiflexion_target);
  }
  // 位置控制命令：move1 <angle> 表示髋关节移动到指定角度（关节角度，度）
  else if (cmd.startsWith("move1 ") || cmd.startsWith("pos1 ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: move1 <angle> (e.g., move1 30.5)");
    } else {
      float angle = cmd.substring(spaceIdx + 1).toFloat();
      // 先读取当前角度
      requestMotorAngle(hipMotor);
      delay(50);  // 等待回复
      {
        CAN_message_t inMsg;
        while (can1.read(inMsg)) {
          handleCanMessage(inMsg);
        }
      }

      // 发送位置控制命令（使用带速度限制的命令，速度限制为30 dps，避免损坏设备）
      sendPositionCommandWithSpeed(hipMotor, angle, 30);
    }
  } else if (cmd.startsWith("move2 ") || cmd.startsWith("pos2 ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: move2 <angle> (e.g., move2 10.5)");
    } else {
      float angle = cmd.substring(spaceIdx + 1).toFloat();
      // 先读取当前角度
      requestMotorAngle(ankleMotor);
      delay(50);  // 等待回复
      {
        CAN_message_t inMsg;
        while (can1.read(inMsg)) {
          handleCanMessage(inMsg);
        }
      }
      // 发送位置控制命令（使用带速度限制的命令，速度限制为30 dps，避免损坏设备）
      sendPositionCommandWithSpeed(ankleMotor, angle, 100);
    }
  }
  // 摆动命令：sw1 10 表示电机1摆动10度
  else if (cmd.startsWith("sw1 ") || cmd.startsWith("swing1 ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: sw1 <amplitude> (e.g., sw1 10)");
    } else {
      float amp = cmd.substring(spaceIdx + 1).toFloat();
      if (amp > 0 && amp <= 90) {
        startSwing(hipSwing, hipMotor, amp);
      } else {
        hostPrintln("ERROR: Amplitude must be 0-90 degrees");
      }
    }
  } else if (cmd.startsWith("sw2 ") || cmd.startsWith("swing2 ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      hostPrintln("ERROR: Usage: sw2 <amplitude> (e.g., sw2 10)");
    } else {
      float amp = cmd.substring(spaceIdx + 1).toFloat();
      if (amp > 0 && amp <= 90) {
        startSwing(ankleSwing, ankleMotor, amp);
      } else {
        hostPrintln("ERROR: Amplitude must be 0-90 degrees");
      }
    }
  } else if (cmd == "stopsw1") {
    stopSwing(hipSwing);
  } else if (cmd == "stopsw2") {
    stopSwing(ankleSwing);
  }
  // 帮助
  else if (cmd == "h" || cmd == "help") {
    hostPrintln("\n=== Command List ===");
    hostPrintln("Enable:  e, e1, e2, enable, enable1, enable2");
    hostPrintln("Disable: d, d1, d2, disable, disable1, disable2");
    hostPrintln("Stop:    stop1, stop2");
    hostPrintln("Clear:   ce, ce1, ce2, clearerror, clearerror1, clearerror2");
    hostPrintln("Read:    r, r1, r2, read, read1, read2");
    hostPrintln("Status:  s, status");
    hostPrintln("Gait:    gc, gc <interval>, gcs (gait collection start/stop)");
    hostPrintln("Bluetooth Telemetry: bt | bt on | bt off");
    hostPrintln("Move:    move1 <angle>, move2 <angle> (e.g., move2 10.5) - move to absolute joint angle");
    hostPrintln("Swing:   sw1 <amp>, sw2 <amp> (e.g., sw1 10)");
    hostPrintln("Stop:    stop1, stop2, stopsw1, stopsw2");
    hostPrintln("Gait Playback: gp <freq> <speed>, gps (gait playback start/stop)");
    hostPrintln("Load Gait: loadgait (load trajectory from JSON)");
    hostPrintln("Ankle Zero: az (ankle zero calibration)");
    hostPrintln("Hip Zero:   hz (hip zero calibration)");
    hostPrintln("Threshold:  th (show adaptive threshold status)");
    hostPrintln("Gait Phase: phase (show gait phase detection status)");
    hostPrintln("4-Phase: ph4 (snapshot), ph4 on/off, ph4 <interval_ms> (realtime stream)");
    hostPrintln("Swing Progress: swing (show swing progress status)");
    hostPrintln("Ankle Assist: assist (show ankle assist strategy status)");
    hostPrintln("Assist On/Off: assiston / assistoff (enable/disable ankle assist)");
    hostPrintln("Ankle Torque Test: ankletorque | ankletorque on <iq> | ankletorque off | ankletorque read");
    hostPrintln("Compliance: compliance (show compliance control status)");
    hostPrintln("Reset Fault: resetfault (reset fault state to normal)");
    hostPrintln("Control Loop: ctrlon / ctrloff (enable/disable 100Hz control loop)");
    hostPrintln("A1 Params: set <name> <value>, get <name>, params (auto-save EEPROM)");
    hostPrintln("Motor Speed: speed <value> / speed (set/query ankle motor speed, 100-10000)");
    hostPrintln("Dorsiflexion: dorsiflex <angle> / df <angle> (set/query ankle dorsiflexion target, 0-40 deg)");
    hostPrintln("Help:    h, help");
  }
  else if (cmd == "p") {
    hostPrintln("=== System Status ===");
    hostPrintf("Control Loop: %s\n", controlLoop.controlEnabled ? "ON" : "OFF");
    hostPrintf("Ankle Assist: %s\n", ankleAssist.enabled ? "ON" : "OFF");
    hostPrintf("Ankle Motor Speed: %u (protocol units)\n", controlLoop.motorSpeed);
    hostPrintf("Ankle Dorsiflexion Target: %.2f deg\n", ankleAssist.dorsiflexion_target);
    hostPrintf("Hip Angle: %.2f deg (logical)\n", getHipDeg());
    hostPrintf("Ankle Angle: %.2f deg (logical)\n", getAnkleDeg());
  }
  else {
    hostPrintf("Unknown command: %s (type 'h' for help)\n", cmd.c_str());
  }
}

// ============================================================================
// 主程序
// ============================================================================

void setup() {
  Serial.begin(115200);
  BT_SERIAL.begin(BT_HC06_BAUD);
  // 当前模式：实时数据仅走蓝牙，USB 保留命令与调试输出
  delay(30);

  hostPrintln("");
  hostPrintln("========================================");
  hostPrintln("Teensy 4.1 Hip-Ankle Motor Controller");
  hostPrintln("CAN Protocol V2.35 Implementation");
  hostPrintln("========================================");
  
  // 初始化 CAN：1Mbps，标准帧
  can1.begin();
  can1.setBaudRate(1000000);  // 1M
  
  hostPrintln("CAN1 initialized at 1 Mbps.");
  hostPrintln("Control ID: 0x140 + MotorID");
  hostPrintln("Feedback ID: Same as Control ID (0x140 + MotorID)");
  hostPrintln("");
  hostPrintln("Protocol Commands:");
  hostPrintln("  0x80: Motor Close (Disable)");
  hostPrintln("  0x81: Motor Stop");
  hostPrintln("  0x88: Motor Run (Enable)");
  hostPrintln("  0x92: Read Multi-turn Angle");
  hostPrintln("  0x9A: Read Status 1");
  hostPrintln("  0x9B: Clear Error");
  hostPrintln("  0x9C: Read Status 2");
  hostPrintln("  0xA3: Position Control 1");
  hostPrintln("  0xA4: Position Control 2 (with speed limit)");
  hostPrintln("");
  hostPrintln("Type 'h' or 'help' for command list.");
  hostPrintln("");

  if (loadA1ParamsFromEeprom()) {
    hostPrintln(">>> Loaded A1 params from EEPROM");
  } else {
    hostPrintln(">>> EEPROM A1 params not found/invalid, using defaults");
  }
  
  // 初始化默认步态轨迹
  initDefaultGaitTrajectory();
  // 启动硬件定时器，直接在定时器回调中调用 runControlLoopOnce，周期 10ms (100Hz)
  controlTimer.begin(runControlLoopOnce, 10000); // 10000 us = 10 ms
  controlTimer.priority(128);
}

// ============================================================================
// 主循环控制（100Hz控制频率）
// ============================================================================

// 控制循环变量定义（结构体定义已移到前面）
// 控制循环变量定义（结构体定义已移到前面）
// controlLoop 定义已移到文件头部


// 控制循环定时器（通过定时器产生节拍，在 loop 中跑控制，避免 ISR 内部做重活）
#include <IntervalTimer.h>

// 启用/禁用控制循环
void setControlLoopEnabled(bool enabled) {
  controlLoop.controlEnabled = enabled;
  if (enabled) {
    controlLoop.lastControlMs = millis();
    controlLoop.controlCount = 0;
    
    // 重置安全状态和滤波器，防止由于上次残留的大电流导致瞬间冲击
    ankleSafety.iq_cmd_prev = 0;
    ankleSafety.compliant = false;
    ankleSafety.in_cooldown = false;
    
    hipSafety.iq_cmd_prev = 0;
    hipSafety.compliant = false;
    hipSafety.in_cooldown = false;
    
    // 重置助力控制器状态
    ankleAssist.initialized = false; // 将触发 updateAnkleAssistStrategy 中的初始化逻辑

    // 重置4相状态机（控制循环重启时从头开始）
    phase4Det.initialized = false;

    // 重置步态检测与滤波
    // hipProcessor.initialized = false; // 可选：是否重置滤波？暂时保留滤波历史可能更好
    
    hostPrintln(">>> Control loop ENABLED (100Hz) - States Reset");
    // 自动启动传感器轮询（喂数据给状态机）
    startSensorPolling(20);  // 默认20ms间隔（50Hz）
  } else {
    hostPrintln(">>> Control loop DISABLED");
    // Timer continues running but runControlLoopOnce() will be no-op when disabled
    // 停止传感器轮询
    stopSensorPolling();
  }
}

// 100Hz 控制循环主体（由定时器驱动，loop 中消费 pending 计数执行）
void runControlLoopOnce() {
  // Run only when enabled
  if (!controlLoop.controlEnabled) {
    return;
  }

  // Enter ISR context: suppress Serial output in called functions
  inIsrContext = true;

  // 在每次控制计算前，优先处理所有 CAN 接收帧，确保获取到最新反馈
  {
    CAN_message_t inMsg;
    // 处理尽可能多的消息，但避免死循环（硬件层面通常会停止返回）
    while (can1.read(inMsg)) {
      handleCanMessage(inMsg);
    }
  }

  uint32_t now = millis();
  controlLoop.lastControlMs = now;
  controlLoop.controlCount++;
  
  // ========================================================================
  // 1. 传感器更新（由传感器轮询定时器统一处理，这里只检查数据新鲜性）
  // ========================================================================
  // 检查传感器数据是否新鲜（500ms内）
  bool hipDataOk = (hipStatus.lastUpdateMs > 0) && 
                   ((now - hipStatus.lastUpdateMs) < COMM_TIMEOUT_MS);
  bool ankleDataOk = (ankleStatus.lastUpdateMs > 0) && 
                     ((now - ankleStatus.lastUpdateMs) < COMM_TIMEOUT_MS);
  
  // ========================================================================
  // 2. 相位识别与 gait 进度
  // ========================================================================
  GaitPhase currentPhase = gaitPhaseDetector.initialized ? 
                           gaitPhaseDetector.currentPhase : PHASE_STANCE;
  float swing_pct = getSwingProgress();   // 0~1
  updateStanceProgress(currentPhase, now);
  float stance_pct = getStancePct(currentPhase, now);
  // 4相检测：必须在stanceProg和swingProgress更新后调用
  updateGaitPhase4Detector();
  float ankle_deg = getAnkleDeg();
  float hip_deg   = getHipDeg();
  updateAnkleVelEstimator(ankle_deg, now);
  float ankle_vel_f = ankleVel.vel_f;
  float hip_vel_f = hipProcessor.hip_vel_f;

  // 相位边沿记录（保持与旧逻辑兼容）
  if (currentPhase != controlLoop.prevPhase) {
    controlLoop.prevPhase = currentPhase;
    // 进入 STANCE 时不再频繁 STOP，只重置 push-off 状态
    if (currentPhase == PHASE_STANCE) {
      pushOff.active = false;
    }
  }
  
  // ========================================================================
  // 3. 计算 iq_target
  // ========================================================================
  // bypass 模式：在目标计算前清除 compliant/cooldown，否则 computeAnkleIqTarget
  // 内部的 !in_cooldown 门控会让 push-off 目标归零，bypass 失效
  if (torqueParams.ankleBypassSafety) {
    ankleSafety.compliant   = false;
    ankleSafety.in_cooldown = false;
  }
  int16_t ankle_iq_target = computeAnkleIqTarget(
      currentPhase, swing_pct, stance_pct,
      ankle_deg, ankle_vel_f, hip_deg, now);

  int16_t hip_iq_target = computeHipIqTarget(
      currentPhase, swing_pct,
      hipProcessor.hip_f, hip_vel_f);

  // 更新 act 标志：有非零踝 iq 视为“位置追踪/助力中”
  controlLoop.anklePositionActive = (ankle_iq_target != 0);

  // ========================================================================
  // 4. 异常检测 & 安全管线
  // ========================================================================
  static float ankle_deg_prev = 0.0f;
  updateAnkleAbnormalDetector(ankle_iq_target, now,
                              ankle_deg, ankle_deg_prev,
                              ankle_vel_f);
  ankle_deg_prev = ankle_deg;

  // 如果检测到异常且尚未进入 compliant，则启动软退出（bypass 模式下跳过）
  if (!torqueParams.ankleBypassSafety &&
      ankleAbn != ABN_NONE && !ankleSafety.compliant && !ankleSafety.in_cooldown) {
    ankleSafety.compliant = true;
    ankleSafety.abnormal_start_ms = now;
  }

  int16_t ankle_iq_cmd;
  if (torqueParams.ankleBypassSafety) {
    // 旁路模式：跳过斜率限幅和 compliant/cooldown，仅保留幅值裁剪
    ankle_iq_cmd = ankle_iq_target;
    if (ankle_iq_cmd >  torqueParams.iq_pf_max) ankle_iq_cmd =  torqueParams.iq_pf_max;
    if (ankle_iq_cmd < -torqueParams.iq_df_max) ankle_iq_cmd = -torqueParams.iq_df_max;
    ankleSafety.iq_cmd_prev = ankle_iq_cmd;  // 保持状态同步，避免退出 bypass 时跳变
  } else {
    ankle_iq_cmd = applySafetyPipeline(
        ankleSafety,
        ankle_iq_target,
        torqueParams.iq_pf_max,  // 正向为 PF
        torqueParams.iq_df_max,  // 负向为 DF
        (ankle_iq_target >= 0) ? torqueParams.diq_up_pf : torqueParams.diq_up_df,
        (ankle_iq_target >= 0) ? torqueParams.diq_dn_pf : torqueParams.diq_dn_df,
        now);
  }

  int16_t hip_iq_cmd = applySafetyPipeline(
      hipSafety,
      hip_iq_target,
      torqueParams.iq_hip_flex_max,
      torqueParams.iq_hip_flex_max,
      torqueParams.diq_up_hip,
      torqueParams.diq_dn_hip,
      now);
  
  // ========================================================================
  // 5. 下发 A1 转矩命令（保持 100Hz，不阻塞）
  // ========================================================================
  if (ankleDataOk) {
    sendTorqueCommand(ankleMotor, ankle_iq_cmd);
  } else {
    ankleSafety.iq_cmd_prev = 0;
  }

  if (hipDataOk) {
    // 若 hipTorqueMode 仍需手动测试，可在此增加优先级判断
    // sendTorqueCommand(hipMotor, hip_iq_cmd);
  } else {
    hipSafety.iq_cmd_prev = 0;
  }

  // 记录调试快照，供 loop 中按需打印
  assistDbg.phase = (int)currentPhase;
  assistDbg.swing_pct = swing_pct;
  assistDbg.stance_pct = stance_pct;
  assistDbg.ankle_deg = ankle_deg;
  assistDbg.ankle_vel_f = ankle_vel_f;
  assistDbg.ankle_iq_target = ankle_iq_target;
  assistDbg.ankle_iq_cmd = ankle_iq_cmd;
  assistDbg.hip_deg = hip_deg;
  assistDbg.hip_vel_f = hip_vel_f;
  assistDbg.hip_iq_target = hip_iq_target;
  assistDbg.hip_iq_cmd = hip_iq_cmd;
  assistDbg.pf = assistFlags.pushOffActive ? 1 : 0;
  assistDbg.df = assistFlags.dfActive ? 1 : 0;
  assistDbg.ul = assistFlags.unloadActive ? 1 : 0;
  assistDbg.comp = ankleSafety.compliant ? 1 : 0;
  assistDbg.cool = ankleSafety.in_cooldown ? 1 : 0;
  assistDbg.abn = (uint8_t)ankleAbn;
  assistDbg.phase4          = (int)getCurrentGaitPhase4();
  assistDbg.phase4_progress = getPhase4Progress();
  assistDbg.phase4_output   = getPhase4ProfileOutput();
  assistDbg.phase4_degraded = isPhase4Degraded() ? 1 : 0;
  assistDbg.lastUpdateMs = now;
  // 退出 ISR 上下文，允许主循环打印
  inIsrContext = false;
}
void loop() {


  // ========================================================================
  // 严重错误处理
  // ========================================================================
  if (isSystemError) {
    if (!errorPrinted) {
      hostPrintln("");
      hostPrintln("**************************************************");
      hostPrintln("              [EMERGENCY STOP]                    ");
      hostPrintf("    Motor %d Reported Error: 0x%02X              \n", errorMotorId, errorCode);
      hostPrintln("    System HALTED. Control Loop Stopped.          ");
      hostPrintln("**************************************************");
      hostPrintln("");
      
      // 尝试发送停止命令给两个电机
      stopMotor(hipMotor);
      stopMotor(ankleMotor);
      
      errorPrinted = true;
    }
    // 错误状态下只保留基本串口命令处理，不运行其他逻辑
  }

  // 处理串口命令（非阻塞）
  processSerialCommand();

  // 更新传感器轮询（在ctrlon开启时自动运行，喂数据给状态机）
  updateSensorPolling();
  // 调试信息打印（保持在 loop 中，避免占用控制节拍）
  // 仅当控制循环启用时打印
  // if (controlLoop.controlEnabled) {
  //   static uint32_t lastDbgMs = 0;
  //   uint32_t now = millis();
  //   if (now - lastDbgMs >= 50 && assistDbg.lastUpdateMs != 0) {
  //     lastDbgMs = now;
  //     if (sensorPolling.enabled) {
  //       Serial.printf("[ASSIST] ph=%d, s=%.3f, st=%.3f, "
  //                     "ank=%.2f, v=%.2f, iqT_a=%d, iqC_a=%d, "
  //                     "hip=%.2f, hipv=%.2f, iqT_h=%d, iqC_h=%d, "
  //                     "flags:PF=%d,DF=%d,UL=%d,comp=%d,cool=%d,abn=%d\n",
  //                     assistDbg.phase, assistDbg.swing_pct * 10.0f, assistDbg.stance_pct * 10.0f,
  //                     assistDbg.ankle_deg, assistDbg.ankle_vel_f, assistDbg.ankle_iq_target, assistDbg.ankle_iq_cmd,
  //                     assistDbg.hip_deg, assistDbg.hip_vel_f, assistDbg.hip_iq_target, assistDbg.hip_iq_cmd,
  //                     assistDbg.pf, assistDbg.df, assistDbg.ul,
  //                     assistDbg.comp, assistDbg.cool, assistDbg.abn);
  //     } else {
  //        Serial.println("[ASSIST-WARN] Sensor Polling DISABLED! (Check control loop or error state)");
  //     }
  //   }
  // }

  // 如果髋关节力矩模式使能，则周期性发送转矩控制命令（例如每50ms）
  if (hipTorqueMode) {
    uint32_t now = millis();
    const uint32_t HIP_TORQUE_PERIOD_MS = 100;
    if (now - hipTorqueLastSendMs >= HIP_TORQUE_PERIOD_MS) {
      hipTorqueLastSendMs = now;
      telemetryPrintf("力矩控制输出。。。\n");
      // 发送转矩控制
      if (!sendTorqueCommand(hipMotor, hipIqTarget)) {
        // 发送失败时简单记录一次错误，避免串口阻塞
        static uint32_t lastHipTorqueErrMs = 0;
        if (now - lastHipTorqueErrMs > 1000) {
          telemetryPrintf("[TX ERROR] Failed to send hip torque command\n");
          lastHipTorqueErrMs = now;
        }
      }
    }
  }

  // 如果踝关节力矩测试模式使能，则周期性发送转矩控制命令
  if (ankleTorqueMode) {
    uint32_t now = millis();
    const uint32_t ANKLE_TORQUE_PERIOD_MS = 100;
    if (now - ankleTorqueLastSendMs >= ANKLE_TORQUE_PERIOD_MS) {
      ankleTorqueLastSendMs = now;
      if (!sendTorqueCommand(ankleMotor, ankleIqTargetManual)) {
        static uint32_t lastAnkleTorqueErrMs = 0;
        if (now - lastAnkleTorqueErrMs > 1000) {
          telemetryPrintf("[TX ERROR] Failed to send ankle torque command\n");
          lastAnkleTorqueErrMs = now;
        }
      }
    }
  }
  
  // 更新摆动（用于调试功能）
  updateSwing(hipSwing);
  updateSwing(ankleSwing);
  
  // 更新步态数据采集
  updateGaitCollection();
  // 更新 4 相步态实时输出
  updatePhase4RealtimeMonitor();
  
  // 更新步态轨迹播放
  updateGaitPlayback();
  
  // ========================================================================
  // 6. 兜底 CAN 读取（当控制循环未开启时执行）
  // ========================================================================
  // 如果控制循环关闭，runControlLoopOnce 不会执行，因此也就不会读取 CAN。
  // 这会导致 gc 指令、s 指令等无法更新数据。
  // 因此在这里补充读取逻辑：
  if (!controlLoop.controlEnabled) {
    CAN_message_t inMsg;
    // 限制每次 loop 最多读多少帧，避免阻塞太久
    int maxReads = 10;
    while (can1.read(inMsg) && maxReads > 0) {
      handleCanMessage(inMsg);
      maxReads--;
    }
  }
}

