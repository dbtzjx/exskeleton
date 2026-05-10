# 调试记录：主控与电机 CAN

本文记录 Teensy 4.1 与髋、踝电机 CAN 通讯相关的调试过程与结论，便于复现与后续迭代。

---

## 2026-05-08 — CAN 统一调度、角度采样与闭环负载

### 背景与现象

- 闭环控制开启（`ctrlon`）后，总线上查询帧与 **A1 转矩帧**叠加，易出现 **CAN 侧积压**：串口诊断里 `[ANGLE_RATE]` 中 **`pend`**（迟到节拍积压）升高，踝侧角度应答 **`ank_rx`** 相对 **`hip_rx`** 更容易掉（甚至接近 0），而 **`tx_fail` 仍为 0**（发送队列未报错，但 RX 侧跟不上）。
- 早期诊断曾出现 **`unif` 极大**：实为首次 `[ANGLE_RATE]` 打印前未对齐统计窗口，把从上电起的周期数累加进第一行；已在固件中对 **`angleDiagPrintIfDue` 首次打印做 priming**，之后 **`unif`** 表示最近 **`ANGLE_DIAG_SERIAL_INTERVAL_MS`**（默认 200 ms）内 **`runUnifiedCanCycle100Hz`** 执行次数，节拍正常时约为 **20**（≈100 Hz×0.2 s）。

### 固件策略（方案概要）

1. **统一 100 Hz CAN 周期** `runUnifiedCanCycle100Hz`（顺序固定）：  
   **`canRxDrain` → `sensorPollingScheduledTx`（角度/STATUS 查询）→ `runControlAlgorithmOnce`（若 `ctrlon`）→ 再次 `canRxDrain` → `angleDiagPrintIfDue`**。  
   目的：先发出查询、再发 A1，并在转矩后再收一轮，减轻应答堆积。
2. **角度查询**：两轴交错，**每轴约 50 Hz**（`ANGLE_HALF_PERIOD_US = 10000`，即 10 ms 半周期交替）；**每统一周期最多发 1 帧角度查询**（`MAX_ANGLE_SENDS_PER_UNIFIED = 1`）。
3. **STATUS 轮询**：非闭环时约 **130 ms**；**闭环开启时拉长为 800 ms**（`STATUS_POLL_INTERVAL_MS_CTRL_ON`），把带宽留给角度应答。
4. **闭环下 A1 转矩降频**：由 **`TORQUE_TX_CTRL_ON_DIVISOR`** 控制 —— 在 `runControlAlgorithmOnce` 内对下发计数取模，**除数为 N 时约为 100/N Hz**（100 Hz 统一节拍前提下）。
5. **ISR**：定时器 ISR 仅递增 **`g_canCycleTicksPending`**（上限 64）；**`loop()`** 内消耗待处理节拍（最多 **48**/次），避免在 ISR 里做 CAN 读写。

### 本次测试结论

- 将 **`TORQUE_TX_CTRL_ON_DIVISOR` 设为 `2`**（A1 约 **50 Hz**）后实测：**髋、踝两路角度均可稳定获取**，满足当前调试需求。
- 若仍过载：可先将除数改为 **`3`**（约 **33 Hz**），或进一步增大 **`STATUS_POLL_INTERVAL_MS_CTRL_ON`**；若助力响应偏钝，在总线稳定前提下可将除数改回 **`2`** 或 **`1`**（须观察 `ank_rx` / `pend`）。

### 串口诊断行格式（简要）

`[ANGLE_RATE] hip_rx=… ank_rx=… tx_hip=… tx_ank=… fail=… unif=… pend=…`

- **`hip_rx` / `ank_rx`**：窗口内角度应答次数折算成的 **Hz**（窗口长度见 `ANGLE_DIAG_SERIAL_INTERVAL_MS`）。
- **`tx_hip` / `tx_ank`**：窗口内角度查询 **TX 尝试**次数。
- **`unif`**：窗口内统一 CAN 周期执行次数（期望约 **20**/200 ms）。
- **`pend`**：`g_canCycleTicksPending`，反映 **loop 是否跟不上 100 Hz 节拍**。

### 相关源码位置

- `firmware/src/main.cpp`：上述常量、`runUnifiedCanCycle100Hz`、`sensorPollingScheduledTx`、`runControlAlgorithmOnce`、`angleDiagPrintIfDue`。

### TODO / 后续可选

- 若协议允许：**仅在“需要刷新力矩”或 iq 非零变化时发 A1**，进一步减负（须对照驱动器对 A1 超时/保持行为）。
- 持续观察 **`ctrlon`** 与 **`gc`/采集** 并存时的总线占用。
