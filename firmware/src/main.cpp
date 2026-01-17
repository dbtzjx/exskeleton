#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <ArduinoJson.h>

// ============================================================================
// CAN 协议配置（根据《电机CAN总线通讯协议 V2.35》）
// ============================================================================

// 使用 Teensy 4.1 的 CAN1 控制器（对应底板 CAN 引脚）
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// 电机配置结构体
struct MotorConfig {
  uint8_t id;          // 协议中的电机 ID（1=髋，2=踝）
  float unitsPerDeg;   // 每 1° 对应的协议单位数
  const char *name;    // 关节名称，便于调试打印
};

// 协议规定：位置和多圈角度的电机轴单位为 0.01°/LSB，即 1° = 100 单位（与具体电机 ID 无关）
// 现在我们希望对外全部使用「关节角度」：
//   - 髋关节：减速比为 1:36，则 1 关节度 = 36 电机轴度 => 1 关节度 = 36 * 100 = 3600 协议单位
//   - 踝关节：减速比为 1:10，则 1 关节度 = 10 电机轴度 => 1 关节度 = 10 * 100 = 1000 协议单位
// 因此：
//   - 髋关节 unitsPerDeg = 3600（1 关节度 = 3600 协议单位，对应电机轴减速比 36）
//   - 踝关节 unitsPerDeg = 1000（1 关节度 = 1000 协议单位，对应电机轴减速比 10）
MotorConfig hipMotor { 1, 3600.0f, "Hip" };
MotorConfig ankleMotor { 2, 1000.0f, "Ankle" };

// 电机状态结构体（用于存储反馈数据）
struct MotorStatus {
  int64_t multiTurnAngle;  // 多圈角度（协议单位，int64_t，单位 0.01°/LSB）
  float angleDeg;          // 转换为角度（度）
  int16_t speed;           // 速度（dps）
  int16_t iq;              // 电流（q轴电流，mA，用于阻力检测）
  int8_t temperature;      // 温度（℃）
  uint8_t motorState;      // 电机状态（0x00=开启，0x10=关闭）
  uint8_t errorState;      // 错误状态
  bool enabled;            // 是否使能
  uint32_t lastUpdateMs;   // 最后更新时间
};

MotorStatus hipStatus = {0, 0.0f, 0, 0, 0, 0, 0, false, 0};
MotorStatus ankleStatus = {0, 0.0f, 0, 0, 0, 0, 0, false, 0};

// 踝关节零点偏移（用于标定）
// 在用户站立自然中立位时，读取的踝电机多圈编码器角度值
// 后续踝解剖角计算：ankle_deg = (pos_raw - ankle_zero_offset) * k_deg
int64_t ankle_zero_offset = 0;  // 协议单位（0.01°/LSB）
bool ankle_zero_calibrated = false;  // 是否已完成标定

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
  
  // 如果幅度太小（<1度），使用默认值避免阈值过小
  if (adaptiveThreshold.hip_amp < 1.0f) {
    adaptiveThreshold.hip_amp = 10.0f;  // 默认幅度10度
  }
  
  // 计算阈值
  // A_up = 0.2 * hip_amp, A_dn = 0.2 * hip_amp
  adaptiveThreshold.A_up = 0.2f * adaptiveThreshold.hip_amp;
  adaptiveThreshold.A_dn = 0.2f * adaptiveThreshold.hip_amp;
  
  // V_up = +20 deg/s, V_dn = -20 deg/s（固定值）
  adaptiveThreshold.V_up = 20.0f;
  adaptiveThreshold.V_dn = -20.0f;
  
  adaptiveThreshold.lastUpdateMs = now;
}

// ============================================================================
// 步态相位识别（两态状态机）
// ============================================================================

// 步态相位枚举
enum GaitPhase {
  PHASE_STANCE = 0,  // 支撑相
  PHASE_SWING = 1    // 摆动相
};

// 步态相位识别状态
struct GaitPhaseDetector {
  GaitPhase currentPhase;         // 当前相位
  uint32_t phaseStartMs;           // 当前相位开始时间（毫秒）
  uint32_t conditionHoldMs;        // 条件持续满足的时间（毫秒）
  bool initialized;                // 是否已初始化
  uint32_t lastUpdateMs;            // 上次更新时间
};

GaitPhaseDetector gaitPhaseDetector = {
  PHASE_STANCE, 0, 0, false, 0
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
  float hip_mean = adaptiveThreshold.hip_mean;
  float A_up = adaptiveThreshold.A_up;
  float A_dn = adaptiveThreshold.A_dn;
  float V_up = adaptiveThreshold.V_up;
  float V_dn = adaptiveThreshold.V_dn;
  
  // 检查相位转换条件
  bool swingConditionMet = (hip_vel_f > V_up) && (hip_f > hip_mean + A_up);
  bool stanceConditionMet = (hip_vel_f < V_dn) && (hip_f < hip_mean - A_dn);
  
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
    }
  }
  
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
  
  // 检测相位切换（在更新t_swing之前，先保存上一次的值）
  float lastSwingDurationSec = 0.0f;
  if (phaseChanged && swingProgress.lastPhase == PHASE_SWING) {
    // 如果从SWING切换到其他相位，保存当前的t_swing值
    // 注意：此时t_swing还是上一次SWING的值（因为还没有更新）
    lastSwingDurationSec = swingProgress.t_swing;
  }
  
  // 计算当前摆动进度
  if (currentPhase == PHASE_SWING) {
    // 当前是摆动相，计算进度
    uint32_t swingDurationMs = getCurrentPhaseDurationMs();
    swingProgress.t_swing = swingDurationMs / 1000.0f;  // 转换为秒
    
    // 计算进度：s = clamp(t_swing / Ts, 0, 1)
    if (swingProgress.Ts > 0.001f) {  // 避免除零
      float progress = swingProgress.t_swing / swingProgress.Ts;
      if (progress < 0.0f) {
        swingProgress.swing_progress = 0.0f;
      } else if (progress > 1.0f) {
        swingProgress.swing_progress = 1.0f;
      } else {
        swingProgress.swing_progress = progress;
      }
    } else {
      swingProgress.swing_progress = 0.0f;
    }
  } else {
    // 当前是支撑相，进度为0
    swingProgress.t_swing = 0.0f;
    swingProgress.swing_progress = 0.0f;
  }
  
  // 处理相位切换（在更新t_swing之后）
  if (phaseChanged) {
    // 相位切换了
    if (swingProgress.lastPhase == PHASE_SWING && currentPhase == PHASE_STANCE) {
      // 从摆动相切换到支撑相：使用保存的摆动时长来更新平均周期
      // 只有当摆动时长合理时才更新平均周期（避免初始化时的错误更新）
      if (lastSwingDurationSec > 0.01f && lastSwingDurationSec < 2.0f) {  // 至少10ms，最多2s
        // 更新平均周期：Ts = 0.8 * Ts + 0.2 * t_swing
        swingProgress.Ts = 0.8f * swingProgress.Ts + 0.2f * lastSwingDurationSec;
        
        // 限制Ts在合理范围内（0.1s ~ 2.0s）
        if (swingProgress.Ts < 0.1f) {
          swingProgress.Ts = 0.1f;
        } else if (swingProgress.Ts > 2.0f) {
          swingProgress.Ts = 2.0f;
        }
      }
    }
    
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
// 踝背屈辅助策略（B人群核心）
// ============================================================================

// 辅助策略参数
#define ANKLE_THETA_LOW  2.0f   // 背屈窗口下限（度）
#define ANKLE_THETA_HIGH 8.0f   // 背屈窗口上限（度）
#define ANKLE_THETA_MIN  -15.0f // 安全限位下限（跖屈，度）
#define ANKLE_THETA_MAX  15.0f  // 安全限位上限（背屈，度）

// 踝背屈辅助状态
struct AnkleAssistController {
  float theta_ref;              // 参考角度（度）
  float theta_target;           // S曲线目标角度（度）
  float assist_factor;          // 助力衰减因子（0.0 ~ 1.0）
  bool enabled;                 // 是否启用辅助
  bool initialized;             // 是否已初始化
};

AnkleAssistController ankleAssist = {
  0.0f,    // theta_ref
  0.0f,    // theta_target
  1.0f,    // assist_factor
  false,   // enabled
  false    // initialized
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
  // 检查前置条件：必须在摆动相且摆动进度已初始化
  if (currentPhase != PHASE_SWING || !swingProgress.initialized) {
    // 不在摆动相，不提供辅助
    ankleAssist.theta_ref = ankle_deg;  // 跟随当前角度
    ankleAssist.theta_target = ANKLE_THETA_LOW;
    ankleAssist.assist_factor = 0.0f;
    return;
  }
  
  // 初始化
  if (!ankleAssist.initialized) {
    ankleAssist.enabled = true;
    ankleAssist.initialized = true;
  }
  
  // 如果辅助未启用，直接返回
  if (!ankleAssist.enabled) {
    ankleAssist.theta_ref = ankle_deg;
    ankleAssist.theta_target = ANKLE_THETA_LOW;
    ankleAssist.assist_factor = 0.0f;
    return;
  }
  
  // 1. 计算S曲线目标角
  // u = s*s*(3 - 2*s)
  float u = smoothStep(swing_progress);
  // θ_target = θ_low + (θ_high - θ_low) * u
  ankleAssist.theta_target = ANKLE_THETA_LOW + (ANKLE_THETA_HIGH - ANKLE_THETA_LOW) * u;
  
  // 2. 窗口辅助核心逻辑
  // if ankle_deg >= θ_low:
  //     θ_ref = ankle_deg       // 不压人，让他自己抬
  // else:
  //     θ_ref = max(ankle_deg, θ_target)
  if (ankle_deg >= ANKLE_THETA_LOW) {
    // 患者已经抬得足够高，不干预
    ankleAssist.theta_ref = ankle_deg;
  } else {
    // 患者抬得不够，提供辅助
    // θ_ref = max(ankle_deg, θ_target)
    ankleAssist.theta_ref = (ankle_deg > ankleAssist.theta_target) ? ankle_deg : ankleAssist.theta_target;
  }
  
  // 3. 安全限位检查
  // 限制参考角度在安全范围内
  if (ankleAssist.theta_ref < ANKLE_THETA_MIN) {
    ankleAssist.theta_ref = ANKLE_THETA_MIN;
  } else if (ankleAssist.theta_ref > ANKLE_THETA_MAX) {
    ankleAssist.theta_ref = ANKLE_THETA_MAX;
  }
  
  // 4. 助力衰减计算
  // p = clamp((ankle_deg - θ_min) / (θ_low - θ_min), 0, 1)
  // assist = 1 - p
  float denominator = ANKLE_THETA_LOW - ANKLE_THETA_MIN;  // θ_low - θ_min = 2 - (-15) = 17
  if (denominator > 0.001f) {
    float p = (ankle_deg - ANKLE_THETA_MIN) / denominator;
    // clamp to [0, 1]
    if (p < 0.0f) {
      p = 0.0f;
    } else if (p > 1.0f) {
      p = 1.0f;
    }
    ankleAssist.assist_factor = 1.0f - p;
  } else {
    ankleAssist.assist_factor = 1.0f;  // 默认最大助力
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

// ============================================================================
// 工具函数
// ============================================================================

// 角度（deg）转换为协议单位（int32）
// 协议单位：0.01°/LSB，即 1° = 100 单位
int32_t angleDegToUnits(const MotorConfig &m, float deg) {
  return static_cast<int32_t>(deg * m.unitsPerDeg);
}

// 协议单位转换为角度（deg）
float unitsToAngleDeg(const MotorConfig &m, int64_t units) {
  return static_cast<float>(units) / m.unitsPerDeg;
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
  
  // 字节1-7：命令数据（如果有）
  if (data != nullptr && dataLen > 0) {
    uint8_t copyLen = (dataLen > 7) ? 7 : dataLen;
    memcpy(&msg.buf[1], data, copyLen);
  }
  
  if (can1.write(msg)) {
    if (printDebug) {
      Serial.printf("[TX] Motor %d, CMD=0x%02X, ID=0x%03X, Data: ", motorId, cmd, msg.id);
      for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", msg.buf[i]);
      }
      Serial.println();
    }
    return true;
  } else {
    if (printDebug) {
      Serial.printf("[TX ERROR] Motor %d, CMD=0x%02X failed\n", motorId, cmd);
    }
    return false;
  }
}

// 使能电机（电机运行命令 0x88）
void enableMotor(const MotorConfig &motor) {
  if (sendCanCommand(motor.id, CMD_MOTOR_RUN)) {
    Serial.printf(">>> %s motor (ID=%d) ENABLED (CMD=0x88)\n", motor.name, motor.id);
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
    Serial.printf(">>> %s motor (ID=%d) DISABLED (CMD=0x80)\n", motor.name, motor.id);
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
  Serial.printf(">>> %s motor (ID=%d) STOPPED (CMD=0x81)\n", motor.name, motor.id);
}

// 读取电机多圈角度（命令 0x92）
void requestMotorAngle(const MotorConfig &motor) {
  sendCanCommand(motor.id, CMD_READ_MULTI_ANGLE, nullptr, 0, false);  // 查询命令不输出TX调试信息
}

// 清除电机错误标志（命令 0x9B）
void clearMotorError(const MotorConfig &motor) {
  sendCanCommand(motor.id, CMD_CLEAR_ERROR);
  Serial.printf(">>> %s motor (ID=%d) error cleared (CMD=0x9B)\n", motor.name, motor.id);
}

// 发送位置控制指令（多圈位置闭环控制命令1，0xA3）
// 协议格式：DATA[0]=0xA3, DATA[1-3]=NULL, DATA[4-7]=位置控制值（int32，小端序）
// 位置控制值单位：0.01°/LSB，即 36000 代表 360°
void sendPositionCommand(const MotorConfig &motor, float targetDeg) {
  // 将角度转换为协议单位（根据用户文档的比例）
  int32_t targetUnits = angleDegToUnits(motor, targetDeg);
  
  // 获取当前角度（用于调试）
  MotorStatus *status = (motor.id == 1) ? &hipStatus : &ankleStatus;
  float currentDeg = status->angleDeg;
  int64_t currentUnits = status->multiTurnAngle;
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
  Serial.printf(">>> %s: current=%.2f deg (%lld units), target=%.2f deg (%ld units), diff=%.2f deg (CMD=0xA3)\n", 
                motor.name, currentDeg, static_cast<long long>(currentUnits), 
                targetDeg, static_cast<long>(targetUnits), diffDeg);
}

// 发送位置控制指令（带速度限制，多圈位置闭环控制命令2，0xA4）
// 协议格式：DATA[0]=0xA4, DATA[1]=NULL, DATA[2-3]=速度限制（uint16，小端序），DATA[4-7]=位置控制值（int32，小端序）
void sendPositionCommandWithSpeed(const MotorConfig &motor, float targetDeg, uint16_t maxSpeed) {
  int32_t targetUnits = angleDegToUnits(motor, targetDeg);
  
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
  
  sendCanCommand(motor.id, CMD_POSITION_CTRL2, data, 7);
  // Serial.printf(">>> Sending position command with speed: %s, target=%.2f deg, maxSpeed=%u dps\n", 
  //               motor.name, targetDeg, maxSpeed);
  // Serial.printf(">>> %s: target = %.2f deg (%ld units), maxSpeed = %u dps (CMD=0xA4)\n", 
  //               motor.name, targetDeg, static_cast<long>(targetUnits), maxSpeed);
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
        
        status->multiTurnAngle = angle;
        
        // 对于踝关节，如果已标定，使用零点偏移计算解剖角度
        if (motor->id == 2 && ankle_zero_calibrated) {
          // 踝解剖角 = (pos_raw - ankle_zero_offset) * k_deg
          // k_deg = 1.0 / unitsPerDeg（将协议单位转换为关节角度）
          int64_t offsetAngle = angle - ankle_zero_offset;
          status->angleDeg = unitsToAngleDeg(*motor, offsetAngle);
        } else {
          // 未标定或髋关节，使用原始角度
          status->angleDeg = unitsToAngleDeg(*motor, angle);
        }
        
        status->lastUpdateMs = millis();
        
        // 对于髋关节，更新信号预处理、自适应阈值、步态相位识别和摆动进度
        if (motor->id == 1) {
          updateHipSignalProcessor(status->angleDeg);
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
              updateAnkleAssistStrategy(ankleStatus.angleDeg, currentPhase, swing_progress);
              
              // 更新顺从控制状态机（需要参考角度、电流、温度、通讯状态）
              float theta_ref = getAnkleReferenceAngle();
              bool commOk = (millis() - ankleStatus.lastUpdateMs) < COMM_TIMEOUT_MS;
              updateComplianceController(ankleStatus.angleDeg, theta_ref, ankleStatus.iq, 
                                         ankleStatus.temperature, commOk);
            }
          }
        }
        
        // 简化输出：只显示角度数据
        if (motor->id == 1) {
          Serial.printf("Hip: %.2f deg\n", status->angleDeg);
        } else {
          if (ankle_zero_calibrated) {
            Serial.printf("Ankle: %.2f deg (calibrated, offset=%lld)\n", 
                         status->angleDeg, static_cast<long long>(ankle_zero_offset));
          } else {
            Serial.printf("Ankle: %.2f deg (raw, NOT calibrated!)\n", status->angleDeg);
          }
        }
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
        
        Serial.printf("[RX] %s: temp=%d℃, voltage=%.2fV, current=%.2fA, state=0x%02X, error=0x%02X, ID=0x%03X\n",
                      motor->name, temperature, voltage * 0.01f, current * 0.01f, 
                      motorState, errorState, msg.id);
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
        
        Serial.printf("[RX] %s: temp=%d℃, iq=%d, speed=%d dps, encoder=%u, ID=0x%03X, CMD=0x%02X\n",
                      motor->name, temperature, iq, speed, encoder, msg.id, cmd);
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
    // 其他帧（可能是心跳、状态帧等）
    Serial.printf("[RX] Unknown ID=0x%03X, DLC=%u, Data: ", msg.id, msg.len);
    for (int i = 0; i < msg.len; i++) {
      Serial.printf("%02X ", msg.buf[i]);
    }
    Serial.println();
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
  swing.centerAngle = status->angleDeg;  // 以当前角度为中心
  swing.amplitude = amplitudeDeg;
  swing.currentAngle = swing.centerAngle;
  swing.direction = true;  // 先向右
  swing.lastStepMs = millis();
  swing.active = true;
  
  Serial.printf(">>> %s swing started: center=%.2f deg, amplitude=%.2f deg\n",
                motor.name, swing.centerAngle, swing.amplitude);
}

void stopSwing(SwingState &swing) {
  swing.active = false;
  Serial.printf(">>> %s swing stopped\n", swing.motor->name);
}

void updateSwing(SwingState &swing) {
  if (!swing.active || swing.motor == nullptr) return;
  
  uint32_t now = millis();
  if (now - swing.lastStepMs < swing.stepIntervalMs) return;
  
  swing.lastStepMs = now;
  
  // 如果幅度为0，不执行摆动
  if (swing.amplitude <= 0.01f) {
    swing.active = false;
    Serial.printf(">>> %s swing stopped: amplitude is zero\n", swing.motor->name);
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
  
  Serial.println(">>> Default gait trajectory initialized");
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
    Serial.printf(">>> Error parsing JSON: %s\n", error.c_str());
    return false;
  }
  
  // 检查必需字段
  if (!doc.containsKey("time") || !doc.containsKey("hip_angle") || !doc.containsKey("ankle_angle")) {
    Serial.println(">>> Error: Missing required fields (time, hip_angle, ankle_angle)");
    return false;
  }
  
  JsonArray timeArray = doc["time"];
  JsonArray hipArray = doc["hip_angle"];
  JsonArray ankleArray = doc["ankle_angle"];
  
  uint16_t pointCount = timeArray.size();
  if (pointCount == 0 || pointCount > MAX_GAIT_POINTS) {
    Serial.printf(">>> Error: Invalid point count: %d (max: %d)\n", pointCount, MAX_GAIT_POINTS);
    return false;
  }
  
  if (hipArray.size() != pointCount || ankleArray.size() != pointCount) {
    Serial.println(">>> Error: Array sizes don't match");
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
  
  Serial.printf(">>> Gait trajectory loaded: %d points, duration=%.2f s\n", 
                pointCount, cycleDuration);
  
  return true;
}

// 开始接收步态数据
void startReceivingGaitData() {
  gaitDataReceive.receiving = true;
  gaitDataReceive.jsonBuffer = "";
  gaitDataReceive.startTime = millis();
  Serial.println(">>> Ready to receive gait data (JSON format)");
  Serial.println(">>> Send JSON data now, timeout: 5 seconds");
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
    Serial.println(">>> Timeout: Gait data reception timeout");
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
      Serial.println(">>> Gait trajectory loaded successfully!");
    } else {
      Serial.println(">>> Failed to load gait trajectory");
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
    Serial.println("Error: Gait trajectory not loaded!");
    return;
  }

  // 如果未显式给出速度（或给的是非正数），根据轨迹和周期自动计算最大速度，
  // 目标是在给定周期内尽量走完完整的轨迹幅度。
  if (maxSpeedDps <= 0.0f) {
    computeRequiredMaxSpeed(frequencyHz, gaitPlayback.maxHipSpeedJoint, gaitPlayback.maxAnkleSpeedJoint);
    Serial.printf(">>> Auto max speed for gait playback (freq=%.2f Hz):\n", frequencyHz);
    Serial.printf(">>>   Hip joint speed: %.1f dps (motor shaft: %.1f dps)\n",
                  gaitPlayback.maxHipSpeedJoint,
                  jointSpeedToMotorSpeed(hipMotor, gaitPlayback.maxHipSpeedJoint));
    Serial.printf(">>>   Ankle joint speed: %.1f dps (motor shaft: %.1f dps)\n",
                  gaitPlayback.maxAnkleSpeedJoint,
                  jointSpeedToMotorSpeed(ankleMotor, gaitPlayback.maxAnkleSpeedJoint));
  } else {
    // 如果显式给出了速度，使用相同的速度（关节速度）
    gaitPlayback.maxHipSpeedJoint = maxSpeedDps;
    gaitPlayback.maxAnkleSpeedJoint = maxSpeedDps;
    Serial.printf(">>> Using specified joint speed: %.1f dps\n", maxSpeedDps);
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
  
  // 保存当前位置作为摆动中心
  gaitPlayback.centerHipAngle = hipStatus.angleDeg;
  gaitPlayback.centerAnkleAngle = ankleStatus.angleDeg;
  
  // 初始化速度平滑器
  gaitPlayback.hipSmoother.currentPosition = hipStatus.angleDeg;
  gaitPlayback.hipSmoother.currentVelocity = 0.0f;
  gaitPlayback.hipSmoother.lastUpdateMs = 0;  // 标记为未初始化
  
  gaitPlayback.ankleSmoother.currentPosition = ankleStatus.angleDeg;
  gaitPlayback.ankleSmoother.currentVelocity = 0.0f;
  gaitPlayback.ankleSmoother.lastUpdateMs = 0;  // 标记为未初始化
  
  gaitPlayback.active = true;
  gaitPlayback.frequency = frequencyHz;
  gaitPlayback.cycleDuration = 1.0f / frequencyHz;
  gaitPlayback.cycleStartMs = millis();
  gaitPlayback.currentPhase = 0.0f;
  
  Serial.printf(">>> Gait playback started: freq=%.2f Hz, duration=%.2f s\n",
                frequencyHz, gaitPlayback.cycleDuration);
  Serial.printf(">>> Center position: Hip=%.2f deg, Ankle=%.2f deg\n",
                gaitPlayback.centerHipAngle, gaitPlayback.centerAnkleAngle);
}

// 停止步态轨迹播放
void stopGaitPlayback() {
  gaitPlayback.active = false;
  Serial.println(">>> Gait playback stopped");
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
// 步态数据采集功能
// ============================================================================

// 步态数据采集状态
struct GaitDataCollection {
  bool enabled;              // 是否启用采集
  uint32_t lastRequestMs;   // 上次请求角度的时间
  uint32_t requestIntervalMs; // 请求间隔（毫秒），建议 10-50ms
  uint32_t lastSendMs;      // 上次发送数据的时间
  uint32_t sendIntervalMs;  // 发送间隔（毫秒），建议 10-50ms
};

GaitDataCollection gaitCollection = {false, 0, 20, 0, 20}; // 默认20ms间隔（50Hz）


// 发送步态数据到串口（JSON格式，便于上位机解析）
// 测试阶段：只输出4个数据：hip_raw(h), hip_f(hf), hip_vel_f(hvf), phase, swing_progress(s)
void sendGaitData() {
  // 格式：{"t":时间戳(ms),"h":髋角度原始值(deg),"hf":滤波髋角(deg),"hvf":滤波髋速度(deg/s),"phase":相位(0=STANCE,1=SWING),"s":摆动进度(0-1)}
  if (hipProcessor.initialized && gaitPhaseDetector.initialized && swingProgress.initialized) {
    // 所有必需模块已初始化，输出完整数据
    Serial.printf("{\"t\":%lu,\"h\":%.2f,\"hf\":%.2f,\"hvf\":%.2f,\"phase\":%d,\"s\":%.3f}\n",
                  millis(),
                  hipStatus.angleDeg,  // hip_raw
                  hipProcessor.hip_f,  // hip_f
                  hipProcessor.hip_vel_f,  // hip_vel_f
                  gaitPhaseDetector.currentPhase,  // phase
                  swingProgress.swing_progress);  // swing_progress
  } else if (hipProcessor.initialized && gaitPhaseDetector.initialized) {
    // 如果信号处理器和相位识别已初始化但摆动进度未初始化
    Serial.printf("{\"t\":%lu,\"h\":%.2f,\"hf\":%.2f,\"hvf\":%.2f,\"phase\":%d,\"s\":0.0}\n",
                  millis(),
                  hipStatus.angleDeg,
                  hipProcessor.hip_f,
                  hipProcessor.hip_vel_f,
                  gaitPhaseDetector.currentPhase);
  } else if (hipProcessor.initialized) {
    // 如果信号处理器已初始化但相位识别未初始化
    Serial.printf("{\"t\":%lu,\"h\":%.2f,\"hf\":%.2f,\"hvf\":%.2f,\"phase\":0,\"s\":0.0}\n",
                  millis(),
                  hipStatus.angleDeg,
                  hipProcessor.hip_f,
                  hipProcessor.hip_vel_f);
  } else {
    // 如果信号处理器未初始化，只发送基本数据（但保持JSON格式完整）
    // 注意：即使模块未初始化，也要输出完整的JSON格式，所有字段都要有
    Serial.printf("{\"t\":%lu,\"h\":%.2f,\"hf\":%.2f,\"hvf\":0.0,\"phase\":0,\"s\":0.0}\n",
                  millis(),
                  hipStatus.angleDeg,
                  hipStatus.angleDeg);  // 如果未初始化，使用原始值作为滤波值
  }
}

// 启动/停止步态数据采集
void startGaitCollection(uint32_t intervalMs = 20) {
  gaitCollection.enabled = true;
  gaitCollection.requestIntervalMs = intervalMs;
  gaitCollection.sendIntervalMs = intervalMs;
  gaitCollection.lastRequestMs = 0;
  gaitCollection.lastSendMs = 0;
  Serial.println(">>> Gait data collection STARTED");
  Serial.printf(">>> Collection interval: %lu ms (%.1f Hz)\n", 
                intervalMs, 1000.0f / intervalMs);
}

void stopGaitCollection() {
  gaitCollection.enabled = false;
  Serial.println(">>> Gait data collection STOPPED");
}

// 更新步态数据采集（在loop中调用）
void updateGaitCollection() {
  if (!gaitCollection.enabled) return;
  
  uint32_t now = millis();
  
  // 定期请求角度（如果距离上次请求超过间隔）
  if (now - gaitCollection.lastRequestMs >= gaitCollection.requestIntervalMs) {
    gaitCollection.lastRequestMs = now;
    requestMotorAngle(hipMotor);
    requestMotorAngle(ankleMotor);
  }
  
  // 定期发送数据到串口（如果距离上次发送超过间隔，且有新数据）
  if (now - gaitCollection.lastSendMs >= gaitCollection.sendIntervalMs) {
    // 检查数据是否更新（避免发送旧数据）
    if (hipStatus.lastUpdateMs > 0 && ankleStatus.lastUpdateMs > 0) {
      gaitCollection.lastSendMs = now;
      sendGaitData();
    }
  }
}

// ============================================================================
// 主循环控制函数前向声明
// ============================================================================

void setControlLoopEnabled(bool enabled);
void updateControlLoop();

// ============================================================================
// 串口命令处理
// ============================================================================

void processSerialCommand() {
  if (!Serial.available()) return;
  
  String line = Serial.readStringUntil('\n');
  line.trim();
  
  if (line.length() == 0) return;
  
  // 如果正在接收步态数据，将数据传递给处理函数
  if (gaitDataReceive.receiving) {
    processReceivedGaitData(line);
    return;
  }
  
  String cmd = line;
  cmd.toLowerCase();
  
  Serial.printf("> Command: %s\n", cmd.c_str());
  
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
    Serial.println("\n=== Motor Status ===");
    Serial.printf("Hip:   angle=%.2f deg (%lld units), speed=%d dps, enabled=%d, state=0x%02X\n",
                  hipStatus.angleDeg, static_cast<long long>(hipStatus.multiTurnAngle),
                  hipStatus.speed, hipStatus.enabled, hipStatus.motorState);
    Serial.printf("Ankle: angle=%.2f deg (%lld units), speed=%d dps, enabled=%d, state=0x%02X\n",
                  ankleStatus.angleDeg, static_cast<long long>(ankleStatus.multiTurnAngle),
                  ankleStatus.speed, ankleStatus.enabled, ankleStatus.motorState);
    
    // 显示髋关节信号预处理状态
    if (hipProcessor.initialized) {
      Serial.println("\n=== Hip Signal Processing ===");
      Serial.printf("Raw angle:     %.2f deg\n", hipStatus.angleDeg);
      Serial.printf("Filtered:      %.2f deg\n", hipProcessor.hip_f);
      Serial.printf("Velocity:      %.2f deg/s\n", hipProcessor.hip_vel);
      Serial.printf("Vel filtered:  %.2f deg/s\n", hipProcessor.hip_vel_f);
    } else {
      Serial.println("\n=== Hip Signal Processing ===");
      Serial.println("Not initialized (need hip angle data)");
    }
    
    // 显示踝关节标定状态
    Serial.println("\n=== Ankle Calibration ===");
    if (ankle_zero_calibrated) {
      Serial.printf("Calibrated: YES (offset=%lld units)\n", 
                   static_cast<long long>(ankle_zero_offset));
    } else {
      Serial.println("Calibrated: NO (use 'az' command to calibrate)");
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
      Serial.println("ERROR: Interval must be 10-1000 ms");
    }
  } else if (cmd == "gcs" || cmd == "gaitstop") {
    stopGaitCollection();
  }
  // 步态轨迹播放命令：
  //   - gp <freq>            ：只指定周期频率，最大速度由程序根据轨迹自动计算，保证尽量走完整幅度
  //   - gp <freq> <speed>    ：指定周期频率和最大速度（高级用法）
  else if (cmd.startsWith("gp ")) {
    int space1 = cmd.indexOf(' ');
    if (space1 < 0) {
      Serial.println("ERROR: Usage: gp <freq> [speed]");
    } else {
      int space2 = cmd.indexOf(' ', space1 + 1);
      // 只有频率参数：gp <freq>，速度自动计算
      if (space2 < 0) {
        float freq = cmd.substring(space1 + 1).toFloat();
        if (freq > 0 && freq <= 5.0f) {
          // 传入 maxSpeedDps = 0，触发自动计算模式
          startGaitPlayback(freq, 0.0f);
        } else {
          Serial.println("ERROR: Frequency must be 0-5 Hz");
        }
      } else {
        // 兼容旧用法：gp <freq> <speed>
        float freq = cmd.substring(space1 + 1, space2).toFloat();
        float speed = cmd.substring(space2 + 1).toFloat();
        if (freq > 0 && freq <= 5.0f && speed > 0 && speed <= 1000.0f) {
          startGaitPlayback(freq, speed);
        } else {
          Serial.println("ERROR: Frequency must be 0-5 Hz, speed must be 0-1000 dps");
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
      Serial.println(">>> Ankle zero calibration SUCCESS");
      Serial.printf(">>> Zero offset: %lld units (%.2f deg)\n", 
                   static_cast<long long>(ankle_zero_offset),
                   unitsToAngleDeg(ankleMotor, ankle_zero_offset));
      Serial.println(">>> Ankle angle will now be calculated relative to this zero position");
      Serial.println(">>> 0 deg = foot at 90° to shank (neutral position)");
    } else {
      Serial.println("ERROR: Failed to read ankle angle. Please try again.");
    }
  }
  // 自适应阈值调试命令：th（threshold）
  else if (cmd == "th" || cmd == "threshold") {
    if (adaptiveThreshold.initialized) {
      Serial.println(">>> Adaptive Threshold Status:");
      Serial.printf(">>>   Window: %d/%d samples\n", adaptiveThreshold.windowCount, HIP_WINDOW_SIZE);
      Serial.printf(">>>   Hip Mean: %.2f deg\n", adaptiveThreshold.hip_mean);
      Serial.printf(">>>   Hip Amplitude: %.2f deg\n", adaptiveThreshold.hip_amp);
      Serial.printf(">>>   A_up: %.2f deg\n", adaptiveThreshold.A_up);
      Serial.printf(">>>   A_dn: %.2f deg\n", adaptiveThreshold.A_dn);
      Serial.printf(">>>   V_up: %.2f deg/s\n", adaptiveThreshold.V_up);
      Serial.printf(">>>   V_dn: %.2f deg/s\n", adaptiveThreshold.V_dn);
      Serial.printf(">>>   T_hold: %lu ms\n", T_HOLD_MS);
      if (hipProcessor.initialized) {
        Serial.printf(">>>   Current hip_f: %.2f deg\n", hipProcessor.hip_f);
        Serial.printf(">>>   Current hip_vel_f: %.2f deg/s\n", hipProcessor.hip_vel_f);
        Serial.printf(">>>   Swing condition: hip_vel_f > V_up && hip_f > hip_mean + A_up\n");
        Serial.printf(">>>   Stance condition: hip_vel_f < V_dn && hip_f < hip_mean - A_dn\n");
      }
    } else {
      Serial.println(">>> Adaptive Threshold: NOT INITIALIZED");
      Serial.println(">>> Start gait collection (gc) to initialize threshold calculation");
    }
  }
  // 步态相位调试命令：phase
  else if (cmd == "phase" || cmd == "gaitphase") {
    if (gaitPhaseDetector.initialized) {
      Serial.println(">>> Gait Phase Detection Status:");
      Serial.printf(">>>   Current Phase: %s\n", 
                   gaitPhaseDetector.currentPhase == PHASE_SWING ? "SWING" : "STANCE");
      Serial.printf(">>>   Phase Duration: %lu ms (%.2f s)\n", 
                   getCurrentPhaseDurationMs(),
                   getCurrentPhaseDurationMs() / 1000.0f);
      Serial.printf(">>>   Condition Hold Time: %lu ms\n", gaitPhaseDetector.conditionHoldMs);
      if (hipProcessor.initialized && adaptiveThreshold.initialized) {
        float hip_f = hipProcessor.hip_f;
        float hip_vel_f = hipProcessor.hip_vel_f;
        float hip_mean = adaptiveThreshold.hip_mean;
        float A_up = adaptiveThreshold.A_up;
        float A_dn = adaptiveThreshold.A_dn;
        float V_up = adaptiveThreshold.V_up;
        float V_dn = adaptiveThreshold.V_dn;
        
        bool swingConditionMet = (hip_vel_f > V_up) && (hip_f > hip_mean + A_up);
        bool stanceConditionMet = (hip_vel_f < V_dn) && (hip_f < hip_mean - A_dn);
        
        Serial.printf(">>>   Current hip_f: %.2f deg\n", hip_f);
        Serial.printf(">>>   Current hip_vel_f: %.2f deg/s\n", hip_vel_f);
        Serial.printf(">>>   hip_mean: %.2f deg\n", hip_mean);
        Serial.printf(">>>   Swing condition (vel>%.1f && angle>%.2f): %s\n", 
                     V_up, hip_mean + A_up, swingConditionMet ? "YES" : "NO");
        Serial.printf(">>>   Stance condition (vel<%.1f && angle<%.2f): %s\n", 
                     V_dn, hip_mean - A_dn, stanceConditionMet ? "YES" : "NO");
      }
      // 显示摆动进度信息
      if (swingProgress.initialized) {
        Serial.println(">>> Swing Progress:");
        Serial.printf(">>>   Ts (avg period): %.3f s\n", swingProgress.Ts);
        Serial.printf(">>>   t_swing (current): %.3f s\n", swingProgress.t_swing);
        Serial.printf(">>>   s (progress): %.3f (%.1f%%)\n", 
                     swingProgress.swing_progress,
                     swingProgress.swing_progress * 100.0f);
      }
    } else {
      Serial.println(">>> Gait Phase Detector: NOT INITIALIZED");
      Serial.println(">>> Start gait collection (gc) to initialize phase detection");
    }
  }
  // 摆动进度调试命令：swing
  else if (cmd == "swing" || cmd == "swingprogress") {
    if (swingProgress.initialized) {
      Serial.println(">>> Swing Progress Status:");
      Serial.printf(">>>   Ts (Average Period): %.3f s\n", swingProgress.Ts);
      Serial.printf(">>>   t_swing (Current Duration): %.3f s\n", swingProgress.t_swing);
      Serial.printf(">>>   s (Progress): %.3f (%.1f%%)\n", 
                   swingProgress.swing_progress,
                   swingProgress.swing_progress * 100.0f);
      if (gaitPhaseDetector.initialized) {
        Serial.printf(">>>   Current Phase: %s\n", 
                     gaitPhaseDetector.currentPhase == PHASE_SWING ? "SWING" : "STANCE");
        Serial.printf(">>>   Phase Duration: %lu ms\n", getCurrentPhaseDurationMs());
      }
    } else {
      Serial.println(">>> Swing Progress: NOT INITIALIZED");
      Serial.println(">>> Start gait collection (gc) to initialize swing progress calculation");
    }
  }
  // 踝背屈辅助策略调试命令：assist
  else if (cmd == "assist" || cmd == "ankleassist") {
    if (ankleAssist.initialized) {
      Serial.println(">>> Ankle Dorsiflexion Assist Strategy Status:");
      Serial.printf(">>>   Enabled: %s\n", ankleAssist.enabled ? "YES" : "NO");
      Serial.printf(">>>   Parameters:\n");
      Serial.printf(">>>     θ_low:  %.2f deg\n", ANKLE_THETA_LOW);
      Serial.printf(">>>     θ_high: %.2f deg\n", ANKLE_THETA_HIGH);
      Serial.printf(">>>     θ_min:  %.2f deg (safety limit)\n", ANKLE_THETA_MIN);
      Serial.printf(">>>     θ_max:  %.2f deg (safety limit)\n", ANKLE_THETA_MAX);
      Serial.printf(">>>   Current Values:\n");
      Serial.printf(">>>     Current Ankle Angle: %.2f deg\n", ankleStatus.angleDeg);
      Serial.printf(">>>     Target Angle (S-curve): %.2f deg\n", ankleAssist.theta_target);
      Serial.printf(">>>     Reference Angle: %.2f deg\n", ankleAssist.theta_ref);
      Serial.printf(">>>     Assist Factor: %.3f (%.1f%%)\n", 
                   ankleAssist.assist_factor,
                   ankleAssist.assist_factor * 100.0f);
      if (gaitPhaseDetector.initialized) {
        GaitPhase currentPhase = getCurrentGaitPhase();
        Serial.printf(">>>   Current Phase: %s\n", 
                     currentPhase == PHASE_SWING ? "SWING (assist active)" : "STANCE (assist inactive)");
        if (currentPhase == PHASE_SWING && swingProgress.initialized) {
          float s = getSwingProgress();
          Serial.printf(">>>   Swing Progress: %.3f (%.1f%%)\n", s, s * 100.0f);
        }
      }
    } else {
      Serial.println(">>> Ankle Assist Strategy: NOT INITIALIZED");
      Serial.println(">>> Start gait collection (gc) to initialize assist strategy");
    }
  }
  // 启用/禁用踝背屈辅助：assiston / assistoff
  else if (cmd == "assiston") {
    setAnkleAssistEnabled(true);
    Serial.println(">>> Ankle dorsiflexion assist ENABLED");
  }
  else if (cmd == "assistoff") {
    setAnkleAssistEnabled(false);
    Serial.println(">>> Ankle dorsiflexion assist DISABLED");
  }
  // 顺从控制调试命令：compliance
  else if (cmd == "compliance" || cmd == "comp") {
    if (complianceCtrl.initialized) {
      Serial.println(">>> Compliance Control Status:");
      const char* stateNames[] = {"NORMAL", "COMPLIANT", "HOLD", "FAULT_SAFE"};
      Serial.printf(">>>   Current State: %s\n", stateNames[complianceCtrl.currentState]);
      Serial.printf(">>>   State Duration: %lu ms\n", getComplianceStateDuration());
      Serial.printf(">>>   Speed Factor: %.2f (%.0f%%)\n", 
                   complianceCtrl.maxSpeedFactor,
                   complianceCtrl.maxSpeedFactor * 100.0f);
      Serial.printf(">>>   Parameters:\n");
      Serial.printf(">>>     I1 (轻度阻力): %d mA\n", COMPLIANCE_I1);
      Serial.printf(">>>     I2 (重度阻力): %d mA\n", COMPLIANCE_I2);
      Serial.printf(">>>     E1 (位置误差1): %.1f deg\n", COMPLIANCE_E1);
      Serial.printf(">>>     E2 (位置误差2): %.1f deg\n", COMPLIANCE_E2);
      Serial.printf(">>>     T_resist: %lu ms\n", COMPLIANCE_T_RESIST);
      Serial.printf(">>>   Current Values:\n");
      Serial.printf(">>>     Ankle Angle: %.2f deg\n", ankleStatus.angleDeg);
      Serial.printf(">>>     Reference Angle: %.2f deg\n", getAnkleReferenceAngle());
      float posError = fabsf(getAnkleReferenceAngle() - ankleStatus.angleDeg);
      Serial.printf(">>>     Position Error: %.2f deg\n", posError);
      Serial.printf(">>>     Current (iq): %d mA\n", ankleStatus.iq);
      float iq_abs = fabsf((float)ankleStatus.iq);
      Serial.printf(">>>     |iq|: %.1f mA\n", iq_abs);
      Serial.printf(">>>     Temperature: %d ℃\n", ankleStatus.temperature);
      bool commOk = (millis() - ankleStatus.lastUpdateMs) < COMM_TIMEOUT_MS;
      Serial.printf(">>>     Communication: %s\n", commOk ? "OK" : "TIMEOUT");
      if (complianceCtrl.currentState == STATE_HOLD) {
        Serial.printf(">>>     Hold Position: %.2f deg\n", complianceCtrl.positionHold);
      }
      if (complianceCtrl.lowResistanceStartMs > 0) {
        uint32_t lowResistDuration = millis() - complianceCtrl.lowResistanceStartMs;
        Serial.printf(">>>     Low Resistance Duration: %lu ms (need %lu ms to exit)\n", 
                     lowResistDuration, COMPLIANCE_T_RESIST);
      }
    } else {
      Serial.println(">>> Compliance Control: NOT INITIALIZED");
      Serial.println(">>> Start gait collection (gc) to initialize compliance control");
    }
  }
  // 重置故障状态：resetfault
  else if (cmd == "resetfault" || cmd == "reset") {
    if (complianceCtrl.currentState == STATE_FAULT_SAFE) {
      resetComplianceFault();
      Serial.println(">>> Fault state RESET to NORMAL");
    } else {
      Serial.println(">>> Current state is not FAULT_SAFE, no reset needed");
      Serial.printf(">>> Current state: %d\n", complianceCtrl.currentState);
    }
  }
  // 启用/禁用控制循环：ctrlon / ctrloff
  else if (cmd == "ctrlon" || cmd == "controlon") {
    setControlLoopEnabled(true);
  }
  else if (cmd == "ctrloff" || cmd == "controloff") {
    setControlLoopEnabled(false);
  }
  // 位置控制命令：move1 <angle> 表示髋关节移动到指定角度（关节角度，度）
  else if (cmd.startsWith("move1 ") || cmd.startsWith("pos1 ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println("ERROR: Usage: move1 <angle> (e.g., move1 30.5)");
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
      Serial.println("ERROR: Usage: move2 <angle> (e.g., move2 10.5)");
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
      Serial.println("ERROR: Usage: sw1 <amplitude> (e.g., sw1 10)");
    } else {
      float amp = cmd.substring(spaceIdx + 1).toFloat();
      if (amp > 0 && amp <= 90) {
        startSwing(hipSwing, hipMotor, amp);
      } else {
        Serial.println("ERROR: Amplitude must be 0-90 degrees");
      }
    }
  } else if (cmd.startsWith("sw2 ") || cmd.startsWith("swing2 ")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println("ERROR: Usage: sw2 <amplitude> (e.g., sw2 10)");
    } else {
      float amp = cmd.substring(spaceIdx + 1).toFloat();
      if (amp > 0 && amp <= 90) {
        startSwing(ankleSwing, ankleMotor, amp);
      } else {
        Serial.println("ERROR: Amplitude must be 0-90 degrees");
      }
    }
  } else if (cmd == "stopsw1") {
    stopSwing(hipSwing);
  } else if (cmd == "stopsw2") {
    stopSwing(ankleSwing);
  }
  // 帮助
  else if (cmd == "h" || cmd == "help") {
    Serial.println("\n=== Command List ===");
    Serial.println("Enable:  e, e1, e2, enable, enable1, enable2");
    Serial.println("Disable: d, d1, d2, disable, disable1, disable2");
    Serial.println("Stop:    stop1, stop2");
    Serial.println("Clear:   ce, ce1, ce2, clearerror, clearerror1, clearerror2");
    Serial.println("Read:    r, r1, r2, read, read1, read2");
    Serial.println("Status:  s, status");
    Serial.println("Gait:    gc, gc <interval>, gcs (gait collection start/stop)");
    Serial.println("Move:    move1 <angle>, move2 <angle> (e.g., move2 10.5) - move to absolute joint angle");
    Serial.println("Swing:   sw1 <amp>, sw2 <amp> (e.g., sw1 10)");
    Serial.println("Stop:    stop1, stop2, stopsw1, stopsw2");
    Serial.println("Gait Playback: gp <freq> <speed>, gps (gait playback start/stop)");
    Serial.println("Load Gait: loadgait (load trajectory from JSON)");
    Serial.println("Ankle Zero: az (ankle zero calibration)");
    Serial.println("Threshold:  th (show adaptive threshold status)");
    Serial.println("Gait Phase: phase (show gait phase detection status)");
    Serial.println("Swing Progress: swing (show swing progress status)");
    Serial.println("Ankle Assist: assist (show ankle assist strategy status)");
    Serial.println("Assist On/Off: assiston / assistoff (enable/disable ankle assist)");
    Serial.println("Compliance: compliance (show compliance control status)");
    Serial.println("Reset Fault: resetfault (reset fault state to normal)");
    Serial.println("Control Loop: ctrlon / ctrloff (enable/disable 100Hz control loop)");
    Serial.println("Help:    h, help");
  }
  else {
    Serial.printf("Unknown command: %s (type 'h' for help)\n", cmd.c_str());
  }
}

// ============================================================================
// 主程序
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // 等待串口连接
  }
  Serial.println("\n========================================");
  Serial.println("Teensy 4.1 Hip-Ankle Motor Controller");
  Serial.println("CAN Protocol V2.35 Implementation");
  Serial.println("========================================");
  
  // 初始化 CAN：1Mbps，标准帧
  can1.begin();
  can1.setBaudRate(1000000);  // 1M
  
  Serial.println("CAN1 initialized at 1 Mbps.");
  Serial.println("Control ID: 0x140 + MotorID");
  Serial.println("Feedback ID: Same as Control ID (0x140 + MotorID)");
  Serial.println("\nProtocol Commands:");
  Serial.println("  0x80: Motor Close (Disable)");
  Serial.println("  0x81: Motor Stop");
  Serial.println("  0x88: Motor Run (Enable)");
  Serial.println("  0x92: Read Multi-turn Angle");
  Serial.println("  0x9A: Read Status 1");
  Serial.println("  0x9B: Clear Error");
  Serial.println("  0x9C: Read Status 2");
  Serial.println("  0xA3: Position Control 1");
  Serial.println("  0xA4: Position Control 2 (with speed limit)");
  Serial.println("\nType 'h' or 'help' for command list.\n");
  
  // 初始化默认步态轨迹
  initDefaultGaitTrajectory();
}

// ============================================================================
// 主循环控制（100Hz控制频率）
// ============================================================================

// 控制循环状态
struct ControlLoop {
  uint32_t lastControlMs;      // 上次控制循环时间（毫秒）
  uint32_t controlIntervalMs;  // 控制周期（毫秒），100Hz = 10ms
  bool controlEnabled;          // 是否启用控制循环
  uint32_t controlCount;        // 控制循环计数（用于调试）
};

ControlLoop controlLoop = {
  0,        // lastControlMs
  10,       // controlIntervalMs (100Hz = 10ms)
  false,    // controlEnabled
  0         // controlCount
};

// 启用/禁用控制循环
void setControlLoopEnabled(bool enabled) {
  controlLoop.controlEnabled = enabled;
  if (enabled) {
    controlLoop.lastControlMs = millis();
    controlLoop.controlCount = 0;
    Serial.println(">>> Control loop ENABLED (100Hz)");
  } else {
    Serial.println(">>> Control loop DISABLED");
  }
}

// 100Hz控制循环（在主循环中调用）
void updateControlLoop() {
  if (!controlLoop.controlEnabled) {
    return;
  }
  
  uint32_t now = millis();
  uint32_t elapsed = now - controlLoop.lastControlMs;
  
  // 检查是否到达控制周期（10ms for 100Hz）
  if (elapsed < controlLoop.controlIntervalMs) {
    return;  // 还没到时间，跳过本次控制
  }
  
  // 更新控制时间戳
  controlLoop.lastControlMs = now;
  controlLoop.controlCount++;
  
  // ========================================================================
  // 1. 传感器更新（已在handleCanMessage中完成，这里确保数据新鲜）
  // ========================================================================
  // 检查传感器数据是否新鲜（500ms内）
  bool hipDataOk = (hipStatus.lastUpdateMs > 0) && 
                   ((now - hipStatus.lastUpdateMs) < COMM_TIMEOUT_MS);
  bool ankleDataOk = (ankleStatus.lastUpdateMs > 0) && 
                     ((now - ankleStatus.lastUpdateMs) < COMM_TIMEOUT_MS);
  
  // 如果数据不新鲜，请求更新
  if (!hipDataOk) {
    requestMotorAngle(hipMotor);
  }
  if (!ankleDataOk) {
    requestMotorAngle(ankleMotor);
    // 同时请求状态2以获取电流数据
    sendCanCommand(ankleMotor.id, CMD_READ_STATUS2, nullptr, 0, false);
  }
  
  // ========================================================================
  // 2. 相位识别（已在handleCanMessage中更新，这里确保已初始化）
  // ========================================================================
  // 相位识别在handleCanMessage中自动更新，这里只需要检查是否已初始化
  
  // ========================================================================
  // 3. 辅助计算（已在handleCanMessage中更新）
  // ========================================================================
  // 辅助策略和顺从控制在handleCanMessage中已更新
  
  // ========================================================================
  // 4. 安全检查（顺从控制状态机已在updateComplianceController中完成）
  // ========================================================================
  // 安全检查已完成，获取当前状态
  ComplianceState compState = getComplianceState();
  float speedFactor = getComplianceSpeedFactor();
  
  // ========================================================================
  // 5. 下发控制命令（仅在辅助启用且数据有效时）
  // ========================================================================
  if (ankleAssist.enabled && ankleAssist.initialized && 
      ankleDataOk && hipDataOk &&
      hipProcessor.initialized && adaptiveThreshold.initialized &&
      gaitPhaseDetector.initialized && swingProgress.initialized &&
      complianceCtrl.initialized) {
    
    // 获取参考角度
    float theta_ref = getAnkleReferenceAngle();
    
    // 根据顺从控制状态调整参考角度
    if (compState == STATE_HOLD) {
      // HOLD状态：保持当前位置
      theta_ref = getComplianceHoldPosition();
    } else if (compState == STATE_FAULT_SAFE) {
      // 故障状态：不发送控制命令（或发送停止命令）
      // 这里可以选择不发送命令，或者发送当前位置保持命令
      theta_ref = ankleStatus.angleDeg;  // 保持当前位置
    }
    
    // 限制参考角度在安全范围内（双重保险）
    if (theta_ref < ANKLE_THETA_MIN) {
      theta_ref = ANKLE_THETA_MIN;
    } else if (theta_ref > ANKLE_THETA_MAX) {
      theta_ref = ANKLE_THETA_MAX;
    }
    
    // 计算速度限制（考虑顺从控制的速度因子和助力衰减）
    // 基础速度限制（关节速度，度/秒）
    float baseMaxSpeedDps = 30.0f;  // 基础最大速度30度/秒
    
    // 应用速度因子（顺从控制）
    float maxSpeedDps = baseMaxSpeedDps * speedFactor;
    
    // 应用助力衰减因子（患者抬得越好，速度越慢）
    float assistFactor = getAnkleAssistFactor();
    maxSpeedDps = maxSpeedDps * (0.3f + 0.7f * assistFactor);  // 速度范围：30% ~ 100%
    
    // 确保速度不为0（至少有一个最小值）
    if (maxSpeedDps < 5.0f) {
      maxSpeedDps = 5.0f;
    }
    
    // 转换为电机轴速度（协议单位：1 dps/LSB）
    uint16_t motorSpeed = jointSpeedToMotorSpeed(ankleMotor, maxSpeedDps);
    
    // 确保速度不为0（至少有一个最小值）
    if (motorSpeed < 5) {
      motorSpeed = 5;
    }
    
    // 发送位置控制命令（带速度限制）
    sendPositionCommandWithSpeed(ankleMotor, theta_ref, motorSpeed);
    
    // 调试输出（每100次控制循环输出一次，避免串口阻塞）
    if (controlLoop.controlCount % 100 == 0) {
      Serial.printf("[CTRL] ref=%.2f deg, speed=%.1f dps (factor=%.2f), state=%d, iq=%d mA\n",
                   theta_ref, maxSpeedDps, speedFactor, compState, ankleStatus.iq);
    }
  }
}

void loop() {
  // 处理串口命令（非阻塞）
  processSerialCommand();
  
  // 轮询接收 CAN 帧（非阻塞）
  {
    CAN_message_t inMsg;
    while (can1.read(inMsg)) {
      handleCanMessage(inMsg);
    }
  }
  
  // 100Hz控制循环（仅在启用时执行）
  updateControlLoop();
  
  // 更新摆动（用于调试功能）
  updateSwing(hipSwing);
  updateSwing(ankleSwing);
  
  // 更新步态数据采集
  updateGaitCollection();
  
  // 更新步态轨迹播放
  updateGaitPlayback();
}
