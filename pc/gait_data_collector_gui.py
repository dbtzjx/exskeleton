#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步态数据采集与可视化程序（GUI版本）
功能：
1. 图形界面操作串口连接
2. 手动发送控制指令
3. 实时绘制步态曲线
4. 识别步态周期并存储最新一个周期
5. 支持数据载入和显示
"""

import serial
import serial.tools.list_ports
import json
import time
import threading
import queue
import matplotlib
matplotlib.use('TkAgg')  # 使用TkAgg后端
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from collections import deque
import numpy as np
from datetime import datetime
import os
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import sys

# ============================================================================
# 配置参数
# ============================================================================

SERIAL_BAUDRATE = 115200
MAX_DATA_POINTS = 2000  # 最大数据点数（用于实时显示）
DATA_FOLDER = "data"  # 数据文件夹
GAIT_CYCLE_FILE = os.path.join(DATA_FOLDER, "gait_cycle_data.json")  # 步态周期数据文件（默认）

# 设置matplotlib中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# ============================================================================
# 数据采集类
# ============================================================================

class GaitDataCollector:
    def __init__(self):
        self.serial_port = None
        self.data_queue = queue.Queue()  # 串口数据队列（统一数据源）
        self.raw_data_queue = queue.Queue()  # 原始数据队列（用于历史记录）
        
        # ========== 数据接收层 ==========
        self.collect_thread = None  # 数据读取线程（连接后自动启动）
        
        # ========== 后处理模块控制 ==========
        self.gait_module_enabled = False  # 步态采集模块开关
        self.hip_module_enabled = False  # 髋关节数据模块开关
        
        # ========== 后处理模块线程 ==========
        self.gait_process_thread = None  # 步态处理线程
        self.hip_process_thread = None  # 髋关节数据处理线程
        
        # 数据存储
        self.time_data = deque(maxlen=MAX_DATA_POINTS)
        self.hip_data = deque(maxlen=MAX_DATA_POINTS)
        self.ankle_data = deque(maxlen=MAX_DATA_POINTS)
        
        # 髋关节信号处理数据存储
        self.hip_filtered_data = deque(maxlen=MAX_DATA_POINTS)  # 滤波后的髋角
        self.hip_velocity_data = deque(maxlen=MAX_DATA_POINTS)  # 髋角速度
        self.hip_velocity_filtered_data = deque(maxlen=MAX_DATA_POINTS)  # 滤波后的髋角速度
        
        # 测试阶段：步态相位和摆动进度数据存储
        self.phase_data = deque(maxlen=MAX_DATA_POINTS)  # 步态相位 (0=STANCE, 1=SWING)
        self.swing_progress_data = deque(maxlen=MAX_DATA_POINTS)  # 摆动进度 (0-1)
        
        # M2阶段：踝关节数据存储
        self.ankle_deg_data = deque(maxlen=MAX_DATA_POINTS)  # 踝关节角度 (deg)
        self.ankle_ref_data = deque(maxlen=MAX_DATA_POINTS)  # 踝关节参考角度 (deg)
        self.act_data = deque(maxlen=MAX_DATA_POINTS)  # Active标志 (1=位置追踪, 0=自由释放)
        
        # 步态周期数据（最新一个周期）
        self.gait_cycle_time = []
        self.gait_cycle_hip = []
        self.gait_cycle_ankle = []
        
        # 步态周期识别（基于峰值检测）
        self.last_hip_angle = None
        self.cycle_start_time = None
        self.cycle_start_hip = None
        self.in_cycle = False
        
        # 峰值检测相关
        self.hip_angle_history = deque(maxlen=20)  # 保存最近20个数据点用于峰值检测
        self.hip_angle_trend = []  # 角度变化趋势（用于检测峰值）
        self.last_peak_time = None  # 上一个峰值的时间
        self.last_peak_angle = None  # 上一个峰值角度
        self.min_cycle_duration = 800  # 最小周期时长（毫秒），正常步态至少0.8秒
        self.max_cycle_duration = 3000  # 最大周期时长（毫秒），防止异常数据
        self.peak_detection_window = 5  # 峰值检测窗口大小（需要前后各5个点确认峰值）
        
        # 性能优化：标记是否载入了外部数据（载入后不需要频繁更新）
        self.is_loaded_data = False
        
        # 数据接收统计（用于调试）
        self.total_received = 0
        self.last_received_time = 0
        
        # ========== 内存管理说明 ==========
        # 所有 deque 缓冲区都使用 maxlen=MAX_DATA_POINTS(2000) 进行自动循环管理
        # 当缓冲区满后，新数据自动覆盖最旧的数据，避免内存泄漏
        # 这种设计在长时间运行时能保持恒定的内存占用（大约 2000*4字节*通道数 ≈ 100KB）
        
    def find_serial_ports(self):
        """查找所有可用串口"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect_serial(self, port, baudrate=115200):
        """连接串口"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            # 清空串口缓冲区
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # 连接后自动启动数据接收线程
            self.start_data_reception()
            
            return True
        except Exception as e:
            raise Exception(f"连接串口失败: {e}")
    
    def disconnect_serial(self):
        """断开串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
    
    def is_connected(self):
        """检查串口是否连接"""
        return self.serial_port is not None and self.serial_port.is_open
    
    def send_command(self, command):
        """发送命令到串口"""
        if not self.is_connected():
            raise Exception("串口未连接")
        
        try:
            command_with_newline = command + '\n'
            self.serial_port.write(command_with_newline.encode('utf-8'))
            self.serial_port.flush()
            
            # 等待并读取返回数据（最多等待500ms）
            import time
            start_time = time.time()
            response_buffer = ""
            while time.time() - start_time < 0.5:  # 最多等待500ms
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    response_buffer += data
                    # 如果收到换行符，说明响应完成
                    if '\n' in response_buffer:
                        break
                time.sleep(0.01)
            
            # 将返回数据添加到原始数据队列
            if response_buffer:
                for line in response_buffer.split('\n'):
                    line = line.strip()
                    if line and not line.startswith('>') and not line.startswith('Command:'):
                        self.raw_data_queue.put(line)
            
            return True
        except Exception as e:
            raise Exception(f"发送命令失败: {e}")
    
    def start_data_reception(self):
        """启动数据接收线程（连接串口后自动调用）"""
        if not self.is_connected():
            raise Exception("串口未连接")
        
        # 如果数据读取线程已经在运行，不重复启动
        if self.collect_thread and self.collect_thread.is_alive():
            return False
        
        # 启动数据读取线程（统一读取串口数据，不进行业务处理）
        self.collect_thread = threading.Thread(target=self._collect_data, daemon=True)
        self.collect_thread.start()
        
        return True
    
    def start_gait_module(self):
        """启动步态采集模块"""
        if self.gait_module_enabled:
            return False  # 已经在运行
        
        self.gait_module_enabled = True
        
        # 启动步态处理线程
        if not self.gait_process_thread or not self.gait_process_thread.is_alive():
            self.gait_process_thread = threading.Thread(target=self._gait_process_loop, daemon=True)
            self.gait_process_thread.start()
        
        return True
    
    def stop_gait_module(self):
        """停止步态采集模块"""
        self.gait_module_enabled = False
        if self.gait_process_thread:
            self.gait_process_thread.join(timeout=2)
    
    def start_hip_module(self):
        """启动髋关节数据模块"""
        if self.hip_module_enabled:
            return False  # 已经在运行
        
        # ✓ 清空旧缓冲区，确保第二次启动时能正确更新
        self.time_data.clear()
        self.hip_data.clear()
        self.hip_filtered_data.clear()
        self.ankle_deg_data.clear()
        self.phase_data.clear()
        self.swing_progress_data.clear()
        self.ankle_ref_data.clear()
        self.act_data.clear()
        
        self.hip_module_enabled = True
        print(f"[模块初始化] 髋关节数据模块启动，已清空缓冲区")
        
        # 启动髋关节数据处理线程
        if not self.hip_process_thread or not self.hip_process_thread.is_alive():
            self.hip_process_thread = threading.Thread(target=self._hip_process_loop, daemon=True)
            self.hip_process_thread.start()
            print(f"[模块状态] 髋关节处理线程已启动 (TID: {self.hip_process_thread.ident})")
        
        return True
    
    def stop_hip_module(self):
        """停止髋关节数据模块"""
        self.hip_module_enabled = False
        if self.hip_process_thread:
            self.hip_process_thread.join(timeout=2)
        
        # ✓ 不再清空缓冲区，保留数据以便用户保存
        # 数据会保留在缓冲区中，直到用户重新启动采集或手动清空
        print(f"[模块状态] 髋关节数据模块已停止，数据已保留（共 {len(self.time_data)} 个数据点）")
    
    def _collect_data(self):
        """数据读取线程（统一从串口读取数据，解析JSON，放入队列）"""
        buffer = ""
        last_diagnostic_time = time.time()
        while self.collect_thread and self.collect_thread.is_alive():
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 按行处理数据
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        original_line = line  # 保存原始行
                        line = line.strip()
                        
                        # 跳过空行
                        if not line:
                            continue
                        
                        # 跳过命令提示（但保留其他所有输出）
                        if line.startswith('>') or line.startswith('Command:'):
                            continue
                        
                        # 记录所有原始返回数据到队列（用于历史记录）
                        self.raw_data_queue.put(original_line)
                        
                        # 解析JSON数据（步态数据）
                        # 尝试解析JSON（可能包含在行的中间，需要查找）
                        if '{' in line and '}' in line:
                            # 提取JSON部分（从第一个{到最后一个}）
                            start_idx = line.find('{')
                            end_idx = line.rfind('}') + 1
                            if start_idx >= 0 and end_idx > start_idx:
                                json_str = line[start_idx:end_idx]
                                try:
                                    data_dict = json.loads(json_str)
                                    # ✓ 检查必要字段（t 和 h）
                                    if 't' in data_dict and 'h' in data_dict:
                                        # ✓ 重点修复：每次数据都放入队列
                                        # 这样所有启用的模块都能看到数据（而不是被竞争消费）
                                        self.data_queue.put(data_dict)
                                        self.total_received += 1
                                        self.last_received_time = time.time()
                                        
                                        # ✓ 定期诊断日志（每1秒一次）
                                        current_time = time.time()
                                        if current_time - last_diagnostic_time > 1.0:
                                            print(f"[_collect_data] 串口接收正常, 已接收 {self.total_received} 个数据点")
                                            print(f"  当前数据: t={data_dict.get('t')}, h={data_dict.get('h'):.2f}, phase={data_dict.get('phase')}, s={data_dict.get('s')}")
                                            print(f"  队列大小: {self.data_queue.qsize()}, gait_enabled={self.gait_module_enabled}, hip_enabled={self.hip_module_enabled}")
                                            last_diagnostic_time = current_time
                                    else:
                                        # ✓ 诊断：打印缺失的字段（每 500 次才打印一次，避免刷屏）
                                        if self.total_received % 500 == 0:
                                            missing = []
                                            if 't' not in data_dict:
                                                missing.append('t')
                                            if 'h' not in data_dict:
                                                missing.append('h')
                                            print(f"[_collect_data] 跳过数据: 缺少字段 {missing}, 接收到字段: {list(data_dict.keys())}")
                                except json.JSONDecodeError as je:
                                    # JSON解析失败，忽略
                                    pass
                
                time.sleep(0.01)  # 避免CPU占用过高
            except Exception as e:
                print(f"[_collect_data] 数据读取错误: {e}")
                time.sleep(0.1)  # 出错时稍长等待
    
    def get_raw_data(self):
        """获取原始返回数据（用于历史记录）"""
        raw_lines = []
        while not self.raw_data_queue.empty():
            try:
                raw_lines.append(self.raw_data_queue.get_nowait())
            except queue.Empty:
                break
        return raw_lines
    
    def _gait_process_loop(self):
        """步态采集模块处理循环（独立线程）"""
        print(f"[_gait_process_loop] 线程启动")
        while self.gait_module_enabled:
            try:
                if not self.data_queue.empty():
                    data = self.data_queue.get(timeout=0.1)
                    # ✓ 修复：使用 .get() 而不是 [] 来访问可选字段
                    timestamp = data.get('t', None)  # 毫秒
                    hip_angle = data.get('h', None)  # 度
                    ankle_angle = data.get('a', None)  # 度
                    
                    # ✓ 验证关键字段存在
                    if timestamp is None or hip_angle is None:
                        print(f"[_gait_process_loop] 跳过无效数据: t={timestamp}, h={hip_angle}")
                        continue  # 跳过无效数据
                    
                    # 添加到实时数据
                    self.time_data.append(timestamp)
                    self.hip_data.append(hip_angle)
                    self.ankle_data.append(ankle_angle)
                    
                    # 步态周期识别（基于髋关节角度）
                    self._detect_gait_cycle(timestamp, hip_angle, ankle_angle)
                else:
                    time.sleep(0.01)  # 队列为空时稍作等待
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[_gait_process_loop] 错误: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.01)
    
    def _hip_process_loop(self):
        """髋关节数据模块处理循环（独立线程）"""
        print(f"[_hip_process_loop] 线程启动")
        data_count = 0
        while self.hip_module_enabled:
            try:
                if not self.data_queue.empty():
                    data = self.data_queue.get(timeout=0.1)
                    
                    # M2阶段：提取数据
                    timestamp = data.get('t', None)  # 毫秒
                    hip_raw = data.get('h', None)   # 髋关节原始角度
                    
                    # ✓ 验证关键字段存在
                    if hip_raw is None or timestamp is None:
                        # 只在启动时打印，避免刷屏
                        continue
                    
                    hip_f = data.get('hf', None)  # 滤波后的髋角
                    hip_vel_f = data.get('hvf', None)  # 滤波后的髋速度
                    phase = data.get('phase', 0)  # 步态相位 (0=STANCE, 1=SWING)
                    swing_progress = data.get('s', 0.0)  # 摆动进度 (0-1)
                    ankle_deg = data.get('a', None)  # 踝关节角度 (deg)
                    ankle_ref = data.get('ar', None)  # 踝关节参考角度 (deg)
                    act = data.get('act', 0)  # Active标志 (1=位置追踪, 0=自由释放)
                    
                    # ✓ 存储数据到缓冲区
                    self.time_data.append(timestamp)
                    self.hip_data.append(hip_raw)  # 存储原始值用于绘图
                    self.hip_filtered_data.append(hip_f if hip_f is not None else None)
                    self.hip_velocity_filtered_data.append(hip_vel_f if hip_vel_f is not None else None)
                    self.phase_data.append(phase)
                    self.swing_progress_data.append(swing_progress)
                    self.ankle_deg_data.append(ankle_deg if ankle_deg is not None else None)
                    self.ankle_ref_data.append(ankle_ref if ankle_ref is not None else None)
                    self.act_data.append(act)
                    
                    data_count += 1
                    # ✓ 每处理50个数据打印一次日志（包含act值）
                    if data_count % 50 == 0:
                        # ✓ 诊断：打印所有缓冲区长度，确认数据被正确添加
                        print(f"[_hip_process_loop] 已处理 {data_count} 个数据点 | 缓冲区长度: time={len(self.time_data)}, hip={len(self.hip_data)}, filtered={len(self.hip_filtered_data)}, ankle={len(self.ankle_deg_data)}, phase={len(self.phase_data)}, swing={len(self.swing_progress_data)}, act={len(self.act_data)}")
                        print(f"  [最新数据] act={act}, phase={phase}, ankle_deg={ankle_deg}")
                    
                    if not self.gait_module_enabled:
                        ankle_angle = ankle_deg if ankle_deg is not None else 0.0
                        self.ankle_data.append(ankle_angle)
                else:
                    time.sleep(0.01)  # 队列为空时稍作等待
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[_hip_process_loop] 错误: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.01)
    
    def _detect_gait_cycle(self, timestamp, hip_angle, ankle_angle):
        """
        检测步态周期（基于峰值检测方法）
        方法：检测髋关节角度的局部峰值（最大值或最小值），两个同类型峰值之间的时间作为一个周期
        优化：使用滑动窗口检测峰值，确保周期时长在合理范围内（0.8-3秒）
        """
        # 初始化
        if self.last_hip_angle is None:
            self.last_hip_angle = hip_angle
            self.cycle_start_time = timestamp
            self.cycle_start_hip = hip_angle
            self.in_cycle = True
            # 重置周期数据
            self.gait_cycle_time = []
            self.gait_cycle_hip = []
            self.gait_cycle_ankle = []
            self.hip_angle_history.clear()
            self.hip_angle_trend.clear()
            return
        
        # 更新历史数据用于峰值检测
        self.hip_angle_history.append({
            'time': timestamp,
            'angle': hip_angle,
            'ankle': ankle_angle
        })
        
        # 计算角度变化趋势（用于检测峰值）
        angle_diff = hip_angle - self.last_hip_angle
        if len(self.hip_angle_trend) > 0:
            # 检测趋势变化（从增加到减少，或从减少到增加）
            last_trend = self.hip_angle_trend[-1]
            if (last_trend > 0 and angle_diff < 0) or (last_trend < 0 and angle_diff > 0):
                # 趋势反转，可能是峰值点
                if len(self.hip_angle_history) >= 3:
                    # 检查是否是局部峰值（前一个点和后一个点都小于/大于当前点）
                    prev_angle = self.hip_angle_history[-3]['angle']
                    next_angle = hip_angle  # 当前点作为候选峰值
                    mid_angle = self.hip_angle_history[-2]['angle']
                    
                    # 检测局部最大值或最小值
                    is_peak = False
                    peak_type = None
                    
                    if mid_angle >= prev_angle and mid_angle >= next_angle:
                        # 局部最大值
                        is_peak = True
                        peak_type = 'max'
                    elif mid_angle <= prev_angle and mid_angle <= next_angle:
                        # 局部最小值
                        is_peak = True
                        peak_type = 'min'
                    
                    if is_peak:
                        peak_time = self.hip_angle_history[-2]['time']
                        peak_angle = self.hip_angle_history[-2]['angle']
                        
                        # 检查是否满足周期条件
                        if self.last_peak_time is not None:
                            cycle_duration = peak_time - self.last_peak_time
                            
                            # 周期时长必须在合理范围内（0.8-3秒）
                            if (self.min_cycle_duration <= cycle_duration <= self.max_cycle_duration):
                                # 检测到完整周期，保存数据
                                if len(self.gait_cycle_time) >= 20:  # 至少20个数据点
                                    self._save_gait_cycle()
                                
                                # 开始新周期（从上一个峰值开始）
                                # 找到上一个峰值在周期数据中的位置
                                start_idx = 0
                                for i, t in enumerate(self.gait_cycle_time):
                                    if t >= self.last_peak_time:
                                        start_idx = i
                                        break
                                
                                # 保留从上一个峰值开始的数据
                                if start_idx < len(self.gait_cycle_time):
                                    self.gait_cycle_time = self.gait_cycle_time[start_idx:]
                                    self.gait_cycle_hip = self.gait_cycle_hip[start_idx:]
                                    self.gait_cycle_ankle = self.gait_cycle_ankle[start_idx:]
                                    self.cycle_start_time = self.last_peak_time
                            
                            # 更新峰值信息（无论是否保存周期）
                            self.last_peak_time = peak_time
                            self.last_peak_angle = peak_angle
                        else:
                            # 第一个峰值，记录并开始周期
                            self.last_peak_time = peak_time
                            self.last_peak_angle = peak_angle
                            self.cycle_start_time = peak_time
                            # 清空之前的数据，从峰值开始
                            peak_idx = 0
                            for i, t in enumerate(self.gait_cycle_time):
                                if t >= peak_time:
                                    peak_idx = i
                                    break
                            if peak_idx > 0:
                                self.gait_cycle_time = self.gait_cycle_time[peak_idx:]
                                self.gait_cycle_hip = self.gait_cycle_hip[peak_idx:]
                                self.gait_cycle_ankle = self.gait_cycle_ankle[peak_idx:]
        
        # 更新趋势
        self.hip_angle_trend.append(angle_diff)
        if len(self.hip_angle_trend) > 5:
            self.hip_angle_trend.pop(0)
        
        # 如果正在周期中，添加数据
        if self.in_cycle:
            self.gait_cycle_time.append(timestamp)
            self.gait_cycle_hip.append(hip_angle)
            self.gait_cycle_ankle.append(ankle_angle)
            
            # 防止周期过长（异常情况）
            if len(self.gait_cycle_time) > 0:
                time_diff = timestamp - self.cycle_start_time
                if time_diff > self.max_cycle_duration:
                    # 超时，重置周期
                    self.in_cycle = False
                    self.gait_cycle_time = []
                    self.gait_cycle_hip = []
                    self.gait_cycle_ankle = []
                    self.last_peak_time = None
                    self.last_peak_angle = None
        else:
            # 开始新周期
            self.in_cycle = True
            self.cycle_start_time = timestamp
            self.cycle_start_hip = hip_angle
            self.gait_cycle_time = [timestamp]
            self.gait_cycle_hip = [hip_angle]
            self.gait_cycle_ankle = [ankle_angle]
            self.last_peak_time = None
            self.last_peak_angle = None
        
        self.last_hip_angle = hip_angle
    
    def _save_gait_cycle(self):
        """保存步态周期数据到文件"""
        if len(self.gait_cycle_time) < 10:  # 至少10个数据点
            return
        
        # 转换为相对时间（从0开始）
        if len(self.gait_cycle_time) > 0:
            start_time = self.gait_cycle_time[0]
            relative_time = [(t - start_time) / 1000.0 for t in self.gait_cycle_time]  # 转换为秒
            
            gait_data = {
                "timestamp": datetime.now().isoformat(),
                "cycle_duration": (self.gait_cycle_time[-1] - self.gait_cycle_time[0]) / 1000.0,  # 秒
                "data_points": len(self.gait_cycle_time),
                "time": relative_time,
                "hip_angle": list(self.gait_cycle_hip),
                "ankle_angle": list(self.gait_cycle_ankle)
            }
            
            # 保存到文件（覆盖）
            try:
                # 确保data文件夹存在
                if not os.path.exists(DATA_FOLDER):
                    os.makedirs(DATA_FOLDER)
                
                with open(GAIT_CYCLE_FILE, 'w', encoding='utf-8') as f:
                    json.dump(gait_data, f, indent=2, ensure_ascii=False)
                print(f"步态周期已保存: {len(self.gait_cycle_time)} 个数据点, "
                      f"周期时长: {gait_data['cycle_duration']:.2f} 秒")
            except Exception as e:
                print(f"保存步态周期数据失败: {e}")
    
    def load_gait_cycle(self, filename=None):
        """载入步态周期数据"""
        if filename is None:
            filename = GAIT_CYCLE_FILE
        
        if not os.path.exists(filename):
            raise Exception(f"文件不存在: {filename}")
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self.gait_cycle_time = data.get('time', [])
            self.gait_cycle_hip = data.get('hip_angle', [])
            self.gait_cycle_ankle = data.get('ankle_angle', [])
            
            # 标记为载入的数据（用于性能优化）
            self.is_loaded_data = True
            
            # 载入外部数据时，重置上次绘制长度，确保只重绘一次
            self._last_cycle_len = -1
            self._last_realtime_len = -1
            
            return True
        except Exception as e:
            raise Exception(f"载入步态周期数据失败: {e}")
    
    def load_realtime_data(self, filename):
        """载入实时数据"""
        if not os.path.exists(filename):
            raise Exception(f"文件不存在: {filename}")
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 将相对时间转换回绝对时间戳（毫秒）
            time_list = data.get('time', [])
            if len(time_list) > 0:
                # 使用当前时间作为基准，或者使用文件中的时间戳
                base_time = int(time.time() * 1000)  # 当前时间戳（毫秒）
                start_offset = int(time_list[0] * 1000)  # 第一个相对时间（转换为毫秒）
                
                # 清空所有数据缓冲区
                self.time_data.clear()
                self.hip_data.clear()
                self.ankle_data.clear()
                self.hip_filtered_data.clear()
                self.ankle_deg_data.clear()
                self.phase_data.clear()
                self.swing_progress_data.clear()
                self.ankle_ref_data.clear()
                self.act_data.clear()
                
                # 获取数据长度
                data_len = len(time_list)
                
                # 加载所有字段，确保长度一致
                hip_angle_list = data.get('hip_angle', [])
                hip_filtered_list = data.get('hip_filtered', [])
                ankle_angle_list = data.get('ankle_angle', [])
                ankle_deg_list = data.get('ankle_deg', [])
                ankle_ref_list = data.get('ankle_ref', [])
                phase_list = data.get('phase', [])
                swing_progress_list = data.get('swing_progress', [])
                act_list = data.get('act', [])
                
                for i, rel_time in enumerate(time_list):
                    timestamp = base_time + int(rel_time * 1000) - start_offset
                    self.time_data.append(timestamp)
                    self.hip_data.append(hip_angle_list[i] if i < len(hip_angle_list) else 0.0)
                    self.ankle_data.append(ankle_angle_list[i] if i < len(ankle_angle_list) else 0.0)
                    # 加载滤波后的髋角（如果存在）
                    self.hip_filtered_data.append(hip_filtered_list[i] if i < len(hip_filtered_list) and hip_filtered_list[i] is not None else None)
                    # 加载踝关节数据（优先使用ankle_deg，否则使用ankle_angle）
                    if i < len(ankle_deg_list) and ankle_deg_list[i] is not None:
                        self.ankle_deg_data.append(ankle_deg_list[i])
                    elif i < len(ankle_angle_list):
                        self.ankle_deg_data.append(ankle_angle_list[i])
                    else:
                        self.ankle_deg_data.append(0.0)
                    # 加载其他字段
                    self.ankle_ref_data.append(ankle_ref_list[i] if i < len(ankle_ref_list) and ankle_ref_list[i] is not None else None)
                    self.phase_data.append(phase_list[i] if i < len(phase_list) else 0)
                    self.swing_progress_data.append(swing_progress_list[i] if i < len(swing_progress_list) and swing_progress_list[i] is not None else 0.0)
                    self.act_data.append(act_list[i] if i < len(act_list) else 0)
            
            # 标记为已载入数据，避免反复重绘导致卡顿
            self.is_loaded_data = True
            
            print(f"[load_realtime_data] 已载入 {len(self.time_data)} 个数据点")
            print(f"  字段长度: time={len(self.time_data)}, hip={len(self.hip_data)}, hip_f={len(self.hip_filtered_data)}, ankle={len(self.ankle_deg_data)}, act={len(self.act_data)}")
            
            return True
        except Exception as e:
            raise Exception(f"载入实时数据失败: {e}")
    
    def clear_all_data(self):
        """清除所有采集的数据"""
        # 清除实时数据
        self.time_data.clear()
        self.hip_data.clear()
        self.ankle_data.clear()
        
        # 清除步态周期数据
        self.gait_cycle_time = []
        self.gait_cycle_hip = []
        self.gait_cycle_ankle = []
        
        # 重置周期检测状态
        self.last_hip_angle = None
        self.cycle_start_time = None
        self.cycle_start_hip = None
        self.in_cycle = False
        self.hip_angle_history.clear()
        self.hip_angle_trend.clear()
        self.last_peak_time = None
        self.last_peak_angle = None
        
        # 清除载入数据标记
        self.is_loaded_data = False
    
    def get_realtime_data(self):
        """获取实时数据（用于绘图，并进行性能优化的数据降采样）"""
        # M2阶段：返回hip_raw、hip_f和ankle_deg用于绘图，phase、swing_progress和ankle_ref单独获取
        # ✓ 诊断：记录何时返回空数据
        if len(self.time_data) == 0 or len(self.hip_data) == 0:
            if not hasattr(self, '_empty_data_logged'):
                self._empty_data_logged = True
                self._empty_data_log_time = time.time()
                # 初次打印时显示所有缓冲区长度
                print(f"[get_realtime_data] ⚠️ 缓冲区为空（初次）:")
                print(f"  time_data={len(self.time_data)}, hip_data={len(self.hip_data)}, hip_filtered={len(self.hip_filtered_data)}, ankle_deg={len(self.ankle_deg_data)}")
                print(f"  hip_module_enabled={self.hip_module_enabled}, gait_module_enabled={self.gait_module_enabled}")
            else:
                current_time = time.time()
                if current_time - self._empty_data_log_time > 5.0:  # 每5秒打印一次
                    print(f"[get_realtime_data] ⚠️ 缓冲区仍为空: time={len(self.time_data)}, hip={len(self.hip_data)}, filtered={len(self.hip_filtered_data)}, ankle={len(self.ankle_deg_data)}")
                    self._empty_data_log_time = current_time
            return [], [], [], [], []
        
        # 确保数据长度一致（取所有相关数据的最小长度）
        min_len = min(len(self.time_data), len(self.hip_data), len(self.hip_filtered_data), len(self.ankle_deg_data), len(self.act_data))
        if min_len == 0:
            if not hasattr(self, '_min_len_zero_logged'):
                self._min_len_zero_logged = True
                self._min_len_zero_log_time = time.time()
                print(f"[get_realtime_data] ⚠️ min_len为0（初次）:")
                print(f"  time={len(self.time_data)}, hip={len(self.hip_data)}, filtered={len(self.hip_filtered_data)}, ankle={len(self.ankle_deg_data)}")
            else:
                current_time = time.time()
                if current_time - self._min_len_zero_log_time > 5.0:
                    print(f"[get_realtime_data] ⚠️ min_len仍为0: time={len(self.time_data)}, hip={len(self.hip_data)}, filtered={len(self.hip_filtered_data)}, ankle={len(self.ankle_deg_data)}")
                    self._min_len_zero_log_time = current_time
            return [], [], [], [], []
        
        # ========== 性能优化：数据降采样 ==========
        # 如果数据点超过 2000 个，进行降采样
        # 这避免matplotlib在绘制超大数据集时卡顿
        if min_len > 2000:
            # 计算降采样步长（例如有2000个点，降采样为1000个点，步长为2）
            step = max(1, min_len // 1000)
            indices = list(range(0, min_len, step))
        else:
            step = 1
            indices = list(range(min_len))
        
        # 转换为相对时间（从最新数据往前）
        latest_time = self.time_data[-1]
        relative_time = [(self.time_data[i] - latest_time) / 1000.0 for i in indices]  # 转换为秒
        hip_data = [self.hip_data[i] for i in indices]  # hip_raw
        
        # 提取滤波后的髋角（确保长度一致）
        hip_filtered = [self.hip_filtered_data[i] if i < len(self.hip_filtered_data) else None for i in indices]
        
        # 提取踝关节角度（确保长度一致）
        ankle_deg = [self.ankle_deg_data[i] if i < len(self.ankle_deg_data) else None for i in indices]
        
        # 提取act数据（确保长度一致）
        act_data = [self.act_data[i] if i < len(self.act_data) else 0 for i in indices]
        
        # ✓ 诊断：每返回100个数据点打印一次
        if not hasattr(self, '_returned_len_log'):
            self._returned_len_log = 0
            self._returned_len_log_time = time.time()
        if len(relative_time) - self._returned_len_log >= 100:
            current_time = time.time()
            print(f"[get_realtime_data] ✓ 返回 {len(relative_time)} 个数据点（共缓存 {len(self.time_data)} 个，hip_data {len(self.hip_data)}, 滤波 {len(self.hip_filtered_data)}）")
            self._returned_len_log = len(relative_time)
        
        return relative_time, hip_data, hip_filtered, ankle_deg, act_data
    
    def get_phase_and_progress(self):
        """获取最新的相位、摆动进度和踝关节参考角度数据（用于数字显示）"""
        phase = self.phase_data[-1] if len(self.phase_data) > 0 else 0
        swing_progress = self.swing_progress_data[-1] if len(self.swing_progress_data) > 0 else 0.0
        ankle_ref = self.ankle_ref_data[-1] if len(self.ankle_ref_data) > 0 and self.ankle_ref_data[-1] is not None else 0.0
        return phase, swing_progress, ankle_ref
    
    def get_signal_processing_data(self):
        """获取信号处理数据（用于调试窗口）"""
        if len(self.time_data) == 0:
            return [], [], [], [], [], []
        
        # 确保所有数据队列长度一致
        min_len = min(len(self.time_data), len(self.hip_data))
        if min_len == 0:
            return [], [], [], [], [], []
        
        # 转换为相对时间（从最新数据往前）
        latest_time = self.time_data[-1]
        relative_time = [(self.time_data[i] - latest_time) / 1000.0 for i in range(min_len)]
        
        # 提取数据
        hip_raw = [self.hip_data[i] for i in range(min_len)]
        
        # 处理滤波数据（确保长度一致）
        hip_filtered = []
        hip_vel = []
        hip_vel_filtered = []
        
        for i in range(min_len):
            if i < len(self.hip_filtered_data):
                hip_filtered.append(self.hip_filtered_data[i])
            else:
                hip_filtered.append(None)
            
            if i < len(self.hip_velocity_data):
                hip_vel.append(self.hip_velocity_data[i])
            else:
                hip_vel.append(None)
            
            if i < len(self.hip_velocity_filtered_data):
                hip_vel_filtered.append(self.hip_velocity_filtered_data[i])
            else:
                hip_vel_filtered.append(None)
        
        return relative_time, hip_raw, hip_filtered, hip_vel, hip_vel_filtered
    
    def get_gait_cycle_data(self):
        """获取步态周期数据（用于绘图）"""
        return (list(self.gait_cycle_time),
                list(self.gait_cycle_hip),
                list(self.gait_cycle_ankle))

# ============================================================================
# GUI主窗口
# ============================================================================

class GaitDataCollectorGUI:
    def __init__(self, root):
        # 初始化周期长度缓存（用于性能优化）
        self._last_cycle_len = -1
        self._last_realtime_len = -1  # 最近一次实时曲线点数，用于避免重复重绘
        self._last_realtime_latest = None  # 最近一次实时曲线的最新时间戳，用于滑动窗口检测
        self.root = root
        self.root.title("步态数据采集与可视化")
        self.root.geometry("1200x800")
        
        self.collector = GaitDataCollector()
        self.update_interval = 50  # 更新间隔（毫秒）
        self.history_max_lines = 500  # 历史记录最大行数
        
        # 按钮状态跟踪
        self.gait_collection_enabled = False  # 步态采集状态
        self.control_loop_enabled = False  # 控制循环启用状态
        
        # 创建界面
        self.create_widgets()
        
        # 启动定时更新（包括串口数据监听）
        self.update_plots()
        self.start_serial_monitor()
    
    def add_history(self, message, direction="TX"):
        """添加指令历史记录"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        if direction == "TX":
            prefix = "[发送]"
            color = "blue"
        elif direction == "RX":
            prefix = "[接收]"
            color = "green"
        else:
            prefix = "[信息]"
            color = "black"
        
        history_line = f"{timestamp} {prefix} {message}\n"
        
        self.history_text.insert(tk.END, history_line)
        
        # 限制历史记录行数
        lines = int(self.history_text.index('end-1c').split('.')[0])
        if lines > self.history_max_lines:
            self.history_text.delete('1.0', f'{lines - self.history_max_lines}.0')
        
        # 自动滚动到底部
        self.history_text.see(tk.END)
    
    def clear_history(self):
        """清空历史记录"""
        self.history_text.delete('1.0', tk.END)
        
    def create_widgets(self):
        """创建GUI组件"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # 左侧控制面板
        control_frame = ttk.LabelFrame(main_frame, text="控制面板", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        main_frame.columnconfigure(0, weight=0)  # 控制面板不扩展
        
        # 串口选择
        ttk.Label(control_frame, text="串口:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, sticky=tk.W, pady=5)
        self.port_combo.bind('<Button-1>', self.refresh_ports)
        
        self.connect_btn = ttk.Button(control_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # 状态显示
        self.status_label = ttk.Label(control_frame, text="状态: 未连接", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=3, sticky=tk.W, pady=5)
        
        # 分隔线
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        # 命令输入
        ttk.Label(control_frame, text="控制命令:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(control_frame, textvariable=self.command_var, width=20)
        self.command_entry.grid(row=3, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        self.command_entry.bind('<Return>', lambda e: self.send_command())
        
        self.send_btn = ttk.Button(control_frame, text="发送", command=self.send_command, state=tk.DISABLED)
        self.send_btn.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 常用命令按钮
        ttk.Label(control_frame, text="常用命令:").grid(row=5, column=0, sticky=tk.W, pady=(10, 5))
        
        cmd_frame = ttk.Frame(control_frame)
        cmd_frame.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 第一行：基础控制
        ttk.Button(cmd_frame, text="使能", command=lambda: self.send_command_text("e"), width=8).grid(row=0, column=0, padx=2)
        ttk.Button(cmd_frame, text="掉电", command=lambda: self.send_command_text("d"), width=8).grid(row=0, column=1, padx=2)
        ttk.Button(cmd_frame, text="读取", command=lambda: self.send_command_text("r"), width=8).grid(row=0, column=2, padx=2)
        ttk.Button(cmd_frame, text="状态", command=lambda: self.send_command_text("s"), width=8).grid(row=0, column=3, padx=2)
        
        # 第二行：采集控制（复用按钮）
        self.gait_collection_btn = ttk.Button(cmd_frame, text="开始采集", command=self.toggle_gait_collection, width=8)
        self.gait_collection_btn.grid(row=1, column=0, padx=2, pady=2)
        
        # 第二行：新增按钮
        ttk.Button(cmd_frame, text="站立初始化", command=self.send_stand_init, width=10).grid(row=1, column=1, padx=2, pady=2)
        self.control_loop_btn = ttk.Button(cmd_frame, text="启用控制循环", command=self.toggle_control_loop, width=12)
        self.control_loop_btn.grid(row=1, column=2, padx=2, pady=2)
        
        # 第三行：重置故障
        ttk.Button(cmd_frame, text="重置故障", command=lambda: self.send_command_text("resetfault"), width=10).grid(row=2, column=0, padx=2, pady=2)
        
        # 分隔线
        ttk.Separator(cmd_frame, orient=tk.HORIZONTAL).grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 后处理模块控制
        ttk.Label(cmd_frame, text="后处理模块:", font=('', 9, 'bold')).grid(row=3, column=0, columnspan=3, sticky=tk.W, pady=(5, 2))
        
        # 步态采集模块控制
        self.gait_module_btn = ttk.Button(cmd_frame, text="步态采集: 关闭", command=self.toggle_gait_module, width=12)
        self.gait_module_btn.grid(row=4, column=0, columnspan=3, padx=2, pady=2, sticky=(tk.W, tk.E))
        
        # 髋关节数据模块控制
        self.hip_module_btn = ttk.Button(cmd_frame, text="髋关节数据: 关闭", command=self.toggle_hip_module, width=12)
        self.hip_module_btn.grid(row=5, column=0, columnspan=3, padx=2, pady=2, sticky=(tk.W, tk.E))
        
        # 数据管理
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=10, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        # 数据管理按钮
        data_btn_frame = ttk.Frame(control_frame)
        data_btn_frame.grid(row=11, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(data_btn_frame, text="载入步态周期", command=self.load_gait_cycle).grid(row=0, column=0, sticky=(tk.W, tk.E), padx=2)
        ttk.Button(data_btn_frame, text="另存为", command=self.save_gait_cycle_as).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=2)
        ttk.Button(data_btn_frame, text="发送到下位机", command=self.send_gait_to_slave).grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=2, pady=2)
        
        data_btn_frame.columnconfigure(0, weight=1)
        data_btn_frame.columnconfigure(1, weight=1)
        
        # 指令收发历史
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=12, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        ttk.Label(control_frame, text="指令收发历史:").grid(row=13, column=0, columnspan=3, sticky=tk.W, pady=(5, 2))
        
        # 创建历史记录文本框（带滚动条）
        history_frame = ttk.Frame(control_frame)
        history_frame.grid(row=14, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        control_frame.rowconfigure(14, weight=1)
        
        self.history_text = tk.Text(history_frame, height=10, width=30, wrap=tk.WORD, font=('Consolas', 9))
        scrollbar = ttk.Scrollbar(history_frame, orient=tk.VERTICAL, command=self.history_text.yview)
        self.history_text.configure(yscrollcommand=scrollbar.set)
        
        self.history_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 清空历史按钮
        ttk.Button(control_frame, text="清空历史", command=self.clear_history).grid(row=15, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 右侧图表区域
        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
        # M2阶段：添加状态显示区域（显示phase、swing_progress和ankle_ref，三个控件放在一行，调小尺寸）
        status_frame = ttk.Frame(plot_frame)
        status_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        # 步态相位显示（调小尺寸）
        ttk.Label(status_frame, text="步态相位:", font=('', 10, 'bold')).grid(row=0, column=0, padx=5, pady=3)
        self.phase_label = tk.Label(status_frame, text="STANCE", font=('Arial', 16, 'bold'), 
                                     bg='#E0E0E0', fg='#0066CC', width=8, relief=tk.RAISED, bd=2)
        self.phase_label.grid(row=0, column=1, padx=5, pady=3)
        
        # 摆动进度显示（调小尺寸）
        ttk.Label(status_frame, text="摆动进度:", font=('', 10, 'bold')).grid(row=0, column=2, padx=5, pady=3)
        self.swing_progress_label = tk.Label(status_frame, text="0.000", font=('Arial', 16, 'bold'), 
                                              bg='#E0E0E0', fg='#CC6600', width=8, relief=tk.RAISED, bd=2)
        self.swing_progress_label.grid(row=0, column=3, padx=5, pady=3)
        
        # 踝关节参考角度显示（调小尺寸）
        ttk.Label(status_frame, text="踝参考角:", font=('', 10, 'bold')).grid(row=0, column=4, padx=5, pady=3)
        self.ankle_ref_label = tk.Label(status_frame, text="0.00", font=('Arial', 16, 'bold'), 
                                         bg='#E0E0E0', fg='#006600', width=8, relief=tk.RAISED, bd=2)
        self.ankle_ref_label.grid(row=0, column=5, padx=5, pady=3)
        
        # 创建matplotlib图表
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        # 右侧 Y 轴：只创建一次，后续重复使用，避免在每次刷新时不断叠加新的坐标轴
        self.ax1_right = self.ax1.twinx()
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        
        # 工具栏
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        self.toolbar.pack(side=tk.LEFT, fill=tk.X)
        
        # 添加功能按钮
        button_frame = ttk.Frame(toolbar_frame)
        button_frame.pack(side=tk.RIGHT, padx=5)
        
        # 显示全部按钮（重置缩放）
        reset_zoom_btn = ttk.Button(button_frame, text="显示全部", command=self.reset_zoom)
        reset_zoom_btn.pack(side=tk.LEFT, padx=2)
        
        # 清除数据按钮
        clear_btn = ttk.Button(button_frame, text="清除数据", command=self.clear_data)
        clear_btn.pack(side=tk.LEFT, padx=2)
        
        # 获取Tkinter widget并绑定滚轮事件（优先级最高）
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # 绑定滚轮事件到Tkinter widget（Windows/Linux）
        canvas_widget.bind('<MouseWheel>', self.on_tk_scroll)
        canvas_widget.bind('<Button-4>', self.on_tk_scroll)  # Linux向上
        canvas_widget.bind('<Button-5>', self.on_tk_scroll)  # Linux向下
        
        # 初始化图表
        self.setup_plots()
        
        # 添加鼠标交互功能（水平方向缩放和移动）
        self.setup_plot_interactions()
        
        # 刷新串口列表
        self.refresh_ports()
    
    def refresh_ports(self, event=None):
        """刷新串口列表"""
        ports = self.collector.find_serial_ports()
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.collector.is_connected():
            # 断开连接
            # 停止所有后处理模块
            if self.collector.gait_module_enabled:
                self.collector.stop_gait_module()
            if self.collector.hip_module_enabled:
                self.collector.stop_hip_module()
            self.collector.disconnect_serial()
            self.connect_btn.config(text="连接")
            self.send_btn.config(state=tk.DISABLED)
            self.status_label.config(text="状态: 未连接", foreground="red")
            self.port_combo.config(state=tk.NORMAL)
            self.add_history("串口已断开", "信息")
        else:
            # 连接串口
            port = self.port_var.get()
            if not port:
                messagebox.showerror("错误", "请选择串口")
                return
            
            try:
                self.collector.connect_serial(port, SERIAL_BAUDRATE)
                self.connect_btn.config(text="断开")
                self.send_btn.config(state=tk.NORMAL)
                self.status_label.config(text=f"状态: 已连接 ({port})", foreground="green")
                self.port_combo.config(state=tk.DISABLED)
                self.add_history(f"串口已连接: {port}", "信息")
                messagebox.showinfo("成功", f"已连接到串口: {port}")
            except Exception as e:
                self.add_history(f"连接失败: {str(e)}", "信息")
                messagebox.showerror("错误", str(e))
    
    def send_command_text(self, command):
        """发送命令文本"""
        self.command_var.set(command)
        self.send_command()
    
    def send_command(self, event=None):
        """发送命令到串口"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "串口未连接")
            return
        
        command = self.command_var.get().strip()
        if not command:
            return
        
        try:
            self.collector.send_command(command)
            # 记录到历史
            self.add_history(command, "TX")
            # 清空输入框
            self.command_var.set("")
        except Exception as e:
            error_msg = f"错误: {str(e)}"
            self.add_history(error_msg, "信息")
            messagebox.showerror("错误", str(e))
    
    
    def load_gait_cycle(self):
        """载入步态周期数据和实时数据"""
        # 选择数据文件夹
        folder_path = filedialog.askdirectory(
            title="选择数据文件夹",
            initialdir=DATA_FOLDER,
            mustexist=True
        )
        
        if not folder_path:
            return  # 用户取消
        
        try:
            # 支持两种命名：gait_cycle.json（新）和 gait_cycle_data.json（旧）
            cycle_candidates = [
                os.path.join(folder_path, "gait_cycle.json"),
                os.path.join(folder_path, "gait_cycle_data.json")
            ]
            realtime_file = os.path.join(folder_path, "realtime_data.json")
            
            loaded_files = []
            
            # 载入步态周期数据（优先使用新文件名，找不到再尝试旧文件名）
            cycle_file = next((f for f in cycle_candidates if os.path.exists(f)), None)
            if cycle_file:
                self.collector.load_gait_cycle(cycle_file)
                loaded_files.append(f"步态周期数据 ({os.path.basename(cycle_file)})")
            
            # 载入实时数据
            if os.path.exists(realtime_file):
                self.collector.load_realtime_data(realtime_file)
                loaded_files.append("实时数据")
                # 重置实时绘制长度缓存，确保仅首次重绘
                self._last_realtime_len = -1
            
            if not loaded_files:
                messagebox.showwarning("警告", "文件夹中没有找到数据文件:\n- gait_cycle.json 或 gait_cycle_data.json\n- realtime_data.json")
                return
            
            # 显示载入成功信息
            message_text = "数据已载入:\n" + "\n".join(loaded_files)
            messagebox.showinfo("成功", message_text)
            self.add_history(f"数据已从 {folder_path} 载入", "信息")
            
            # 强制更新图表（清除缓存，确保重绘）
            self._last_cycle_len = -1
            self._last_realtime_len = -1
            if hasattr(self, '_plot_initialized'):
                self._plot_initialized = False  # 强制重新初始化绘制
            # 更新图表
            self.update_plots()
            
        except Exception as e:
            messagebox.showerror("错误", str(e))
            self.add_history(f"载入数据失败: {str(e)}", "信息")
    
    def send_gait_to_slave(self):
        """将步态周期数据发送到下位机"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "串口未连接，请先连接串口。")
            return
        
        # 检查是否有步态周期数据
        if len(self.collector.gait_cycle_time) == 0:
            messagebox.showwarning("警告", "没有步态周期数据，请先载入数据。")
            return
        
        try:
            self.add_history("正在发送步态数据到下位机...", "信息")
            
            # 准备JSON数据
            gait_data = {
                "data_type": "gait_cycle",
                "cycle_duration": self.collector.gait_cycle_time[-1] if len(self.collector.gait_cycle_time) > 0 else 0.0,
                "data_points": len(self.collector.gait_cycle_time),
                "time": self.collector.gait_cycle_time,
                "hip_angle": self.collector.gait_cycle_hip,
                "ankle_angle": self.collector.gait_cycle_ankle
            }
            
            # 转换为JSON字符串（紧凑格式，无缩进）
            json_str = json.dumps(gait_data, ensure_ascii=False, separators=(',', ':'))
            
            # 发送 loadgait 命令（自动记录到历史）
            self.collector.send_command("loadgait")
            self.add_history("loadgait", "TX")
            
            # 等待下位机准备接收（100ms）
            time.sleep(0.1)
            
            # 发送JSON数据（一次性发送，串口缓冲区应该足够）
            self.collector.serial_port.write((json_str + '\n').encode('utf-8'))
            self.collector.serial_port.flush()
            self.add_history(f"步态数据JSON ({len(json_str)} 字节)", "TX")
            
            # 等待下位机处理（响应会通过串口监听线程自动记录到历史）
            time.sleep(0.6)  # 等待下位机处理和响应
            
            # 检查响应队列中是否有成功消息（响应会被自动记录到历史）
            success = False
            temp_raw_lines = []
            while not self.collector.raw_data_queue.empty():
                try:
                    line = self.collector.raw_data_queue.get_nowait()
                    temp_raw_lines.append(line)
                    # 检查下位机的成功消息（可能包含 >>> 前缀）
                    if "Gait trajectory loaded" in line or "trajectory loaded" in line.lower() or "loaded:" in line.lower():
                        success = True
                except queue.Empty:
                    break
            
            # 将检查过的响应重新放回队列，以便update_plots自动记录到历史
            for line in temp_raw_lines:
                self.collector.raw_data_queue.put(line)
            
            if success:
                self.add_history(f"步态数据加载成功: {len(self.collector.gait_cycle_time)} 个数据点, 周期时长 {gait_data['cycle_duration']:.2f} 秒", "信息")
                messagebox.showinfo("成功", f"步态数据已发送到下位机\n数据点数: {len(self.collector.gait_cycle_time)}\n周期时长: {gait_data['cycle_duration']:.2f} 秒")
            else:
                self.add_history("提示: 步态数据已发送，请查看下位机响应", "信息")
                messagebox.showinfo("提示", f"步态数据已发送到下位机\n数据点数: {len(self.collector.gait_cycle_time)}\n周期时长: {gait_data['cycle_duration']:.2f} 秒\n\n请查看指令历史中的下位机响应")
            
        except Exception as e:
            error_msg = f"发送步态数据失败: {str(e)}"
            messagebox.showerror("错误", error_msg)
            self.add_history(error_msg, "信息")
    
    def reset_zoom(self):
        """重置图表缩放，显示全部曲线"""
        # 重置第一个图表（实时数据）
        self.ax1.relim()
        self.ax1.autoscale()
        if hasattr(self, "ax1_right") and self.ax1_right is not None:
            self.ax1_right.relim()
            self.ax1_right.autoscale()
        
        # 重置第二个图表（步态周期）
        self.ax2.relim()
        self.ax2.autoscale()
        
        # 更新图表显示
        self.canvas.draw()
        self.add_history("图表已重置为显示全部", "信息")
    
    def clear_data(self):
        """清除所有采集的数据和曲线"""
        # 确认操作
        if messagebox.askyesno("确认", "确定要清除所有采集的数据和曲线吗？"):
            # 清除数据
            self.collector.clear_all_data()
            # 重置周期长度缓存
            self._last_cycle_len = -1
            # 重置缩放
            self.reset_zoom()
            # 异步更新图表（避免阻塞主线程）
            self.root.after(0, self._force_update_plots)
            self.add_history("已清除所有采集数据", "信息")
    
    def _force_update_plots(self):
        """强制更新图表（不安排下一次定时更新）"""
        # 处理新数据（但限制处理数量，避免阻塞）
        max_process = 50  # 最多处理50条数据
        count = 0
        while not self.collector.data_queue.empty() and count < max_process:
            try:
                data = self.collector.data_queue.get_nowait()
                timestamp = data['t']
                hip_angle = data['h']
                ankle_angle = data['a']
                
                self.collector.time_data.append(timestamp)
                self.collector.hip_data.append(hip_angle)
                self.collector.ankle_data.append(ankle_angle)
                count += 1
            except queue.Empty:
                break
        
        # 更新实时数据图
        xlim1 = self.ax1.get_xlim() if len(self.ax1.lines) > 0 else None
        auto_scale_x1 = (xlim1 is None or xlim1 == (0.0, 1.0))
        
        # 清空左右 Y 轴
        self.ax1.clear()
        if hasattr(self, "ax1_right") and self.ax1_right is not None:
            self.ax1_right.clear()
        else:
            # 兼容性：如果尚未创建，则创建一次
            self.ax1_right = self.ax1.twinx()

        time_data, hip_data, hip_filtered, ankle_data, act_data = self.collector.get_realtime_data()
        if len(time_data) > 0 and len(hip_data) > 0 and len(ankle_data) > 0 and len(time_data) == len(hip_data) == len(ankle_data):
            # 绘制角度数据（使用左Y轴）
            self.ax1.plot(time_data, hip_data, 'b-', label='髋关节', linewidth=1.5)
            
            # 绘制踝关节曲线：不管act是0还是1，都使用橙黄色
            if len(act_data) == len(time_data) and len(act_data) == len(ankle_data) and len(act_data) > 0:
                # 简化绘制：直接绘制整个踝关节曲线，使用橙黄色
                ankle_data_valid = [(a if a is not None else float('nan')) for a in ankle_data]
                self.ax1.plot(time_data, ankle_data_valid, color='#FFA500', linewidth=1.5, label='踝关节', alpha=0.9)
            else:
                # 如果act数据不完整，也使用橙黄色绘制
                ankle_data_valid = [(a if a is not None else float('nan')) for a in ankle_data]
                self.ax1.plot(time_data, ankle_data_valid, color='#FFA500', linewidth=1.5, label='踝关节', alpha=0.9)
            
            # 绘制髋关节滤波后的角度
            hip_filtered_valid = [(x if x is not None else float('nan')) for x in hip_filtered]
            if any(not np.isnan(x) for x in hip_filtered_valid):
                self.ax1.plot(time_data, hip_filtered_valid, 'b--', label='髋关节(滤波)', linewidth=1.2, alpha=0.7)
            
            # 获取速度数据
            _, _, _, hip_vel, hip_vel_filtered = self.collector.get_signal_processing_data()
            
            # 绘制髋关节速度
            if len(hip_vel) == len(time_data):
                hip_vel_valid = [(x if x is not None else float('nan')) for x in hip_vel]
                if any(not np.isnan(x) for x in hip_vel_valid):
                    self.ax1_right.plot(time_data, hip_vel_valid, 'g-', label='髋速度', linewidth=1.2, alpha=0.8)
            
            # 绘制髋关节滤波后的速度
            if len(hip_vel_filtered) == len(time_data):
                hip_vel_filtered_valid = [(x if x is not None else float('nan')) for x in hip_vel_filtered]
                if any(not np.isnan(x) for x in hip_vel_filtered_valid):
                    self.ax1_right.plot(time_data, hip_vel_filtered_valid, 'g--', label='髋速度(滤波)', linewidth=1.2, alpha=0.7)
            
            self.ax1.set_title('实时数据（髋关节和踝关节角度，髋关节速度）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度 (度)', color='black')
            self.ax1_right.set_ylabel('速度 (度/秒)', color='green')
            self.ax1_right.tick_params(axis='y', labelcolor='green')
            self.ax1.grid(True)
            
            # 合并图例
            lines1, labels1 = self.ax1.get_legend_handles_labels()
            lines2, labels2 = self.ax1_right.get_legend_handles_labels()
            self.ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
            if not auto_scale_x1 and xlim1 is not None:
                self.ax1.set_xlim(xlim1)
            else:
                self.ax1.relim()
                self.ax1.autoscale()
        else:
            self.ax1.text(0.5, 0.5, '等待数据...', 
                         horizontalalignment='center', verticalalignment='center',
                         transform=self.ax1.transAxes, fontsize=14)
            self.ax1.set_title('实时数据（髋关节和踝关节角度，髋关节速度）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度 (度)')
            self.ax1.grid(True)
        
        # 更新步态周期图
        xlim2 = self.ax2.get_xlim() if len(self.ax2.lines) > 0 else None
        auto_scale_x2 = (xlim2 is None or xlim2 == (0.0, 1.0))
        
        self.ax2.clear()
        cycle_time, cycle_hip, cycle_ankle = self.collector.get_gait_cycle_data()
        if len(cycle_time) > 0 and len(cycle_hip) > 0 and len(cycle_ankle) > 0:
            self.ax2.plot(cycle_time, cycle_hip, 'b-', label='髋关节', linewidth=2)
            self.ax2.plot(cycle_time, cycle_ankle, 'r-', label='踝关节', linewidth=2)
            self.ax2.set_title('最新步态周期', fontsize=12)
            self.ax2.set_xlabel('时间 (秒)')
            self.ax2.set_ylabel('角度 (度)')
            self.ax2.grid(True)
            self.ax2.legend(loc='upper right')
            if not auto_scale_x2 and xlim2 is not None:
                self.ax2.set_xlim(xlim2)
            else:
                self.ax2.relim()
                self.ax2.autoscale()
        else:
            self.ax2.text(0.5, 0.5, '暂无步态周期数据\n（等待数据采集...）', 
                         horizontalalignment='center', verticalalignment='center',
                         transform=self.ax2.transAxes, fontsize=12)
            self.ax2.set_title('最新步态周期', fontsize=12)
            self.ax2.set_xlabel('时间 (秒)')
            self.ax2.set_ylabel('角度 (度)')
            self.ax2.grid(True)
        
        # 更新周期长度缓存
        self._last_cycle_len = len(cycle_time)
        
        # 使用draw_idle避免阻塞
        self.canvas.draw_idle()
    
    def save_gait_cycle_as(self):
        """另存为：保存当前步态数据（步态周期数据和实时数据）"""
        # 检查是否有数据可保存（检查所有可能的数据字段）
        has_cycle_data = (len(self.collector.gait_cycle_time) > 0 and 
                         len(self.collector.gait_cycle_hip) > 0 and 
                         len(self.collector.gait_cycle_ankle) > 0)
        
        # 检查实时数据：只要time_data有数据，就认为有实时数据可保存
        has_realtime_data = len(self.collector.time_data) > 0
        
        if not has_cycle_data and not has_realtime_data:
            messagebox.showwarning("警告", "没有可保存的数据\n\n提示：请先启动数据采集，然后再保存数据")
            return
        
        # 选择保存文件夹
        folder_name = f"gait_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        default_path = os.path.join(DATA_FOLDER, folder_name)
        
        # 使用文件夹选择对话框
        folder_path = filedialog.askdirectory(
            title="选择保存文件夹",
            initialdir=DATA_FOLDER,
            mustexist=False
        )
        
        if not folder_path:
            return  # 用户取消
        
        # 如果用户选择了已存在的文件夹，使用它；否则创建新文件夹
        if not os.path.exists(folder_path):
            try:
                os.makedirs(folder_path)
            except Exception as e:
                messagebox.showerror("错误", f"创建文件夹失败: {str(e)}")
                return
        
        try:
            saved_files = []
            
            # 保存步态周期数据
            if has_cycle_data:
                cycle_file = os.path.join(folder_path, "gait_cycle.json")
                if len(self.collector.gait_cycle_time) > 0:
                    start_time = self.collector.gait_cycle_time[0]
                    relative_time = [(t - start_time) / 1000.0 for t in self.collector.gait_cycle_time]
                else:
                    relative_time = []
                
                cycle_data = {
                    "timestamp": datetime.now().isoformat(),
                    "data_type": "gait_cycle",
                    "cycle_duration": (self.collector.gait_cycle_time[-1] - self.collector.gait_cycle_time[0]) / 1000.0 if len(self.collector.gait_cycle_time) > 1 else 0.0,
                    "data_points": len(self.collector.gait_cycle_time),
                    "time": relative_time,
                    "hip_angle": list(self.collector.gait_cycle_hip),
                    "ankle_angle": list(self.collector.gait_cycle_ankle)
                }
                
                with open(cycle_file, 'w', encoding='utf-8') as f:
                    json.dump(cycle_data, f, indent=2, ensure_ascii=False)
                saved_files.append(f"步态周期数据: {cycle_file} ({cycle_data['data_points']} 点)")
            
            # 保存实时数据（包含所有采集的字段）
            if has_realtime_data:
                realtime_file = os.path.join(folder_path, "realtime_data.json")
                if len(self.collector.time_data) > 0:
                    start_time = self.collector.time_data[0]
                    relative_time = [(t - start_time) / 1000.0 for t in self.collector.time_data]
                else:
                    relative_time = []
                
                # 确定数据长度（以time_data为准）
                data_len = len(self.collector.time_data)
                
                # 构建完整的实时数据字典
                realtime_data = {
                    "timestamp": datetime.now().isoformat(),
                    "data_type": "realtime",
                    "duration": (self.collector.time_data[-1] - self.collector.time_data[0]) / 1000.0 if len(self.collector.time_data) > 1 else 0.0,
                    "data_points": data_len,
                    "time": relative_time,
                    "hip_angle": list(self.collector.hip_data)[:data_len] if len(self.collector.hip_data) > 0 else [],
                    "hip_filtered": [x if x is not None else None for x in list(self.collector.hip_filtered_data)[:data_len]] if len(self.collector.hip_filtered_data) > 0 else [],
                    "ankle_angle": list(self.collector.ankle_data)[:data_len] if len(self.collector.ankle_data) > 0 else [],
                    "ankle_deg": list(self.collector.ankle_deg_data)[:data_len] if len(self.collector.ankle_deg_data) > 0 else [],
                    "ankle_ref": [x if x is not None else None for x in list(self.collector.ankle_ref_data)[:data_len]] if len(self.collector.ankle_ref_data) > 0 else [],
                    "phase": list(self.collector.phase_data)[:data_len] if len(self.collector.phase_data) > 0 else [],
                    "swing_progress": [x if x is not None else None for x in list(self.collector.swing_progress_data)[:data_len]] if len(self.collector.swing_progress_data) > 0 else [],
                    "act": list(self.collector.act_data)[:data_len] if len(self.collector.act_data) > 0 else []
                }
                
                with open(realtime_file, 'w', encoding='utf-8') as f:
                    json.dump(realtime_data, f, indent=2, ensure_ascii=False)
                saved_files.append(f"实时数据: {realtime_file} ({realtime_data['data_points']} 点)")
            
            # 显示保存成功信息
            message_text = "数据已保存到文件夹:\n" + folder_path + "\n\n" + "\n".join(saved_files)
            messagebox.showinfo("成功", message_text)
            self.add_history(f"数据已保存到: {folder_path}", "信息")
            
        except Exception as e:
            error_msg = f"保存数据失败: {str(e)}"
            messagebox.showerror("错误", error_msg)
            self.add_history(error_msg, "信息")
    
    def setup_plots(self):
        """设置图表"""
        # 测试阶段：只显示髋关节原始值和滤波值
        self.ax1.set_title('实时数据（髋关节原始值和滤波值）', fontsize=12)
        self.ax1.set_xlabel('时间 (秒)')
        self.ax1.set_ylabel('角度 (度)')
        self.ax1.grid(True)
        
        self.ax2.set_title('最新步态周期', fontsize=12)
        self.ax2.set_xlabel('时间 (秒)')
        self.ax2.set_ylabel('角度 (度)')
        self.ax2.grid(True)
        
        self.fig.tight_layout()
    
    def setup_plot_interactions(self):
        """设置图表交互功能（缩放和移动）"""
        # 鼠标状态
        self.pan_active = False
        self.zoom_active = False
        self.press_x = None
        self.press_y = None
        self.xlim_backup = None
        self.ylim_backup = None
        
        # 连接鼠标事件（优先级高于工具栏默认行为）
        self.scroll_cid = self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas.mpl_connect('button_press_event', self.on_press)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_motion)
    
    def on_press(self, event):
        """鼠标按下事件"""
        if event.inaxes is None:
            return
        
        # 右键或中键：开始平移
        if event.button == 3 or event.button == 2:  # 右键或中键
            self.pan_active = True
            self.press_x = event.xdata
            self.press_y = event.ydata
            # 保存当前视图范围
            if event.inaxes == self.ax1:
                self.xlim_backup = self.ax1.get_xlim()
                self.ylim_backup = self.ax1.get_ylim()
            elif event.inaxes == self.ax2:
                self.xlim_backup = self.ax2.get_xlim()
                self.ylim_backup = self.ax2.get_ylim()
    
    def on_release(self, event):
        """鼠标释放事件"""
        self.pan_active = False
        self.zoom_active = False
        self.press_x = None
        self.press_y = None
    
    def on_motion(self, event):
        """鼠标移动事件（用于平移）"""
        if not self.pan_active or event.inaxes is None:
            return
        
        if self.press_x is None or self.xlim_backup is None:
            return
        
        # 计算移动距离（只使用水平方向）
        dx = event.xdata - self.press_x
        
        # 更新X轴范围（水平移动）
        if event.inaxes == self.ax1:
            xlim = self.xlim_backup
            x_range = xlim[1] - xlim[0]
            new_xlim = (xlim[0] - dx, xlim[1] - dx)
            self.ax1.set_xlim(new_xlim)
            # Y轴保持不变
            self.ax1.set_ylim(self.ylim_backup)
        elif event.inaxes == self.ax2:
            xlim = self.xlim_backup
            x_range = xlim[1] - xlim[0]
            new_xlim = (xlim[0] - dx, xlim[1] - dx)
            self.ax2.set_xlim(new_xlim)
            # Y轴保持不变
            self.ax2.set_ylim(self.ylim_backup)
        
        self.canvas.draw_idle()
    
    def on_tk_scroll(self, event):
        """Tkinter滚轮事件处理（直接绑定到widget）"""
        # 获取Tkinter widget
        canvas_widget = self.canvas.get_tk_widget()
        
        # 获取鼠标在canvas中的坐标（相对于canvas widget）
        x = event.x
        y = event.y
        
        # 确定鼠标在哪个轴上
        # 将Tkinter坐标转换为matplotlib display坐标
        # 获取figure的bbox（display坐标）
        fig_bbox = self.fig.bbox
        if fig_bbox is None:
            return
        
        # 获取canvas widget的尺寸
        widget_width = canvas_widget.winfo_width()
        widget_height = canvas_widget.winfo_height()
        
        if widget_width == 1 or widget_height == 1:  # 未初始化
            return
        
        # 计算figure在widget中的位置和缩放
        fig_width = fig_bbox.width
        fig_height = fig_bbox.height
        
        # 转换为figure的display坐标（像素，相对于figure左下角）
        # 注意：Tkinter的y坐标是从上到下，matplotlib的display坐标是从下到上
        fig_x = (x / widget_width) * fig_width
        fig_y = fig_height - (y / widget_height) * fig_height  # 翻转y坐标
        
        # 检查在哪个axes中（使用axes的bbox来检查）
        ax = None
        ax1_bbox = self.ax1.bbox
        ax2_bbox = self.ax2.bbox
        
        if ax1_bbox and ax2_bbox:
            # 检查点是否在ax1的bbox内
            ax1_x0, ax1_y0 = ax1_bbox.x0, ax1_bbox.y0
            ax1_x1, ax1_y1 = ax1_bbox.x1, ax1_bbox.y1
            
            # 检查点是否在ax2的bbox内
            ax2_x0, ax2_y0 = ax2_bbox.x0, ax2_bbox.y0
            ax2_x1, ax2_y1 = ax2_bbox.x1, ax2_bbox.y1
            
            if ax1_x0 <= fig_x <= ax1_x1 and ax1_y0 <= fig_y <= ax1_y1:
                ax = self.ax1
            elif ax2_x0 <= fig_x <= ax2_x1 and ax2_y0 <= fig_y <= ax2_y1:
                ax = self.ax2
            else:
                return
        else:
            return
        
        # 将display坐标转换为数据坐标
        inv = ax.transData.inverted()
        xdata, ydata = inv.transform((fig_x, fig_y))
        
        # 获取当前轴范围
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        # 确定滚动方向
        # Windows: event.delta (正数=向上, 负数=向下)
        # Linux: event.num (4=向上, 5=向下)
        if hasattr(event, 'delta'):
            if event.delta > 0:
                scale_factor = 1.15  # 放大
            elif event.delta < 0:
                scale_factor = 1.0 / 1.15  # 缩小
            else:
                return
        elif hasattr(event, 'num'):
            if event.num == 4:
                scale_factor = 1.15  # 放大
            elif event.num == 5:
                scale_factor = 1.0 / 1.15  # 缩小
            else:
                return
        else:
            return
        
        # X轴缩放（以鼠标位置为中心）
        x_range = xlim[1] - xlim[0]
        if x_range <= 0:
            return
        new_x_range = x_range * scale_factor
        center_ratio_x = (xdata - xlim[0]) / x_range
        new_xlim = (
            xdata - center_ratio_x * new_x_range,
            xdata + (1 - center_ratio_x) * new_x_range
        )
        
        # 应用新的X轴范围
        ax.set_xlim(new_xlim)
        
        # Y轴根据数据自适应（重新计算）
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)
        
        # 立即更新显示
        self.canvas.draw_idle()
    
    def on_scroll(self, event):
        """matplotlib滚轮事件处理（备用）"""
        # 如果不在图表区域内，忽略
        if event.inaxes is None:
            return
        
        # 如果工具栏处于缩放或平移模式，让工具栏处理
        if hasattr(self.toolbar, '_active') and self.toolbar._active in ['ZOOM', 'PAN']:
            return
        
        # 获取当前轴
        if event.inaxes == self.ax1:
            ax = self.ax1
            xlim = self.ax1.get_xlim()
        elif event.inaxes == self.ax2:
            ax = self.ax2
            xlim = self.ax2.get_xlim()
        else:
            return
        
        # 获取鼠标位置
        xdata = event.xdata
        if xdata is None:
            return
        
        # 计算缩放比例（向上滚动放大，向下滚动缩小）
        # event.step: 正数=向上, 负数=向下 (更可靠的方式)
        # event.button: 4=向上, 5=向下 (备用方式)
        if hasattr(event, 'step'):
            if event.step > 0:
                scale_factor = 1.15  # 放大
            elif event.step < 0:
                scale_factor = 1.0 / 1.15  # 缩小
            else:
                return
        elif event.button == 'up' or event.button == 4:
            scale_factor = 1.15  # 放大
        elif event.button == 'down' or event.button == 5:
            scale_factor = 1.0 / 1.15  # 缩小
        else:
            return
        
        # X轴缩放（以鼠标位置为中心）
        x_range = xlim[1] - xlim[0]
        if x_range <= 0:
            return
        new_x_range = x_range * scale_factor
        center_ratio_x = (xdata - xlim[0]) / x_range
        new_xlim = (
            xdata - center_ratio_x * new_x_range,
            xdata + (1 - center_ratio_x) * new_x_range
        )
        
        # 应用新的X轴范围
        ax.set_xlim(new_xlim)
        
        # Y轴根据数据自适应（重新计算）
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)
        
        # 立即更新显示
        self.canvas.draw_idle()
        
        # 阻止事件继续传播（避免工具栏的默认处理）
        return True
    
    def update_plots(self):
        """更新图表（使用增量更新而非完全重绘，显著提升性能）"""
        # 更新采集状态显示
        total_points = len(self.collector.hip_data)
        queue_size = self.collector.data_queue.qsize()
        
        # 更新标签信息（低成本操作，不影响绘制）
        phase, swing_progress, ankle_ref = self.collector.get_phase_and_progress()
        if hasattr(self, 'phase_label'):
            phase_text = "SWING" if phase == 1 else "STANCE"
            phase_color = '#CC6600' if phase == 1 else '#0066CC'
            self.phase_label.config(text=phase_text, fg=phase_color)
        if hasattr(self, 'swing_progress_label'):
            self.swing_progress_label.config(text=f"{swing_progress:.3f}")
        if hasattr(self, 'ankle_ref_label'):
            self.ankle_ref_label.config(text=f"{ankle_ref:.2f}")
        
        # 获取实时数据
        time_data, hip_data, hip_filtered, ankle_deg, act_data = self.collector.get_realtime_data()
        
        # ========== 性能优化1：仅在数据显著变化时重绘 ==========
        # 使用 total_received 计数判断是否有新数据到达（不受环形缓冲长度限制）
        new_len = len(time_data)
        if not hasattr(self, '_last_received_count'):
            self._last_received_count = 0
        data_received_increment = self.collector.total_received - self._last_received_count

        # 初始化绘制状态
        if not hasattr(self, '_plot_initialized'):
            self._plot_initialized = False
            self._last_realtime_len = 0
            self._plot_lines = {'hip_raw': None, 'hip_f': None, 'ankle_deg': None, 'act': None}

        # 判断是否需要重绘：首次绘制、收到新数据、或载入了外部数据
        need_full_redraw = False
        # 如果载入了数据且_last_realtime_len为-1，强制重绘
        is_loaded_data = hasattr(self.collector, 'is_loaded_data') and self.collector.is_loaded_data
        if not self._plot_initialized or data_received_increment > 0 or (is_loaded_data and self._last_realtime_len == -1):
            need_full_redraw = True
            # 绘制第一个图表（实时数据）
            if new_len > 0:
                if need_full_redraw or not self._plot_initialized:
                    # 完全重绘：清空并重新绘制所有曲线
                    self.ax1.clear()
                    self._plot_lines = {}
                    
                    # 确保数据长度一致
                    min_len = min(len(time_data), len(hip_data), 
                                len(hip_filtered) if hip_filtered else 0, 
                                len(ankle_deg) if ankle_deg else 0,
                                len(act_data) if act_data else 0)
                    if min_len > 0:
                        # 截取到最小长度
                        time_arr = np.array(list(time_data)[-min_len:])
                        hip_arr = np.array(list(hip_data)[-min_len:])
                        
                        # 绘制原始髋关节角度
                        line_hip, = self.ax1.plot(time_arr, hip_arr, 'b-', 
                                                 label='髋关节原始(hip_raw)', linewidth=1.5)
                        self._plot_lines['hip_raw'] = line_hip
                        
                        # 绘制滤波后的髋角（如果有效）
                        if hip_filtered and len(hip_filtered) > 0:
                            valid_indices = [i for i, x in enumerate(hip_filtered[-min_len:]) if x is not None]
                            if len(valid_indices) > 0:
                                valid_time = time_arr[valid_indices]
                                valid_filtered = np.array([hip_filtered[i] for i in valid_indices])
                                line_f, = self.ax1.plot(valid_time, valid_filtered, 'r--', 
                                                       label='髋关节滤波(hip_f)', linewidth=1.5, alpha=0.8)
                                self._plot_lines['hip_f'] = line_f
                        
                        # 绘制踝关节角度（如果有效）
                        if ankle_deg and len(ankle_deg) > 0:
                            valid_indices = [i for i, x in enumerate(ankle_deg[-min_len:]) if x is not None]
                            if len(valid_indices) > 0:
                                valid_time = time_arr[valid_indices]
                                valid_ankle = np.array([ankle_deg[i] for i in valid_indices])
                                line_a, = self.ax1.plot(valid_time, valid_ankle, 'g-', 
                                                       label='踝关节角度(ankle_deg)', linewidth=1.5, alpha=0.8)
                                self._plot_lines['ankle_deg'] = line_a
                        
                        # 绘制act逻辑量（橙黄色实线）
                        if act_data and len(act_data) > 0:
                            # 将act值（0或1）映射到Y轴范围，便于观察
                            # 可以映射到角度范围，例如：act=1时显示为60度，act=0时显示为-10度
                            act_mapped = np.array([60.0 if a == 1 else -10.0 for a in act_data[-min_len:]])
                            line_act, = self.ax1.plot(time_arr, act_mapped, color='#FFA500', 
                                                     linestyle='-', linewidth=1.5, 
                                                     label='Act(位置追踪=1,释放=0)', alpha=0.9)
                            self._plot_lines['act'] = line_act
                        
                        self.ax1.set_title('实时数据（髋关节原始值、滤波值和踝关节角度）', fontsize=12)
                        self.ax1.set_xlabel('时间 (秒)')
                        self.ax1.set_ylabel('角度 (度)', color='black')
                        self.ax1.grid(True, alpha=0.3)
                        self.ax1.legend(loc='upper right', fontsize=9)
                        self.ax1.relim()
                        self.ax1.autoscale()
                        
                        self._plot_initialized = True
                        self._last_realtime_len = new_len
                        self._last_received_count = self.collector.total_received
                        # 如果载入了数据，重置标记，避免下次重复重绘
                        if is_loaded_data:
                            self.collector.is_loaded_data = False
                        
                        # ✓ 首次初始化完成后立即绘制，确保曲线可见
                        self.canvas.draw_idle()
                        print(f"[update_plots] ✓ 首次绘制完成，已调用 draw_idle()")
                else:
                    # 增量更新：仅更新数据而不清空重绘
                    if self._plot_lines and 'hip_raw' in self._plot_lines:
                        min_len = min(len(time_data), len(hip_data), 
                                    len(hip_filtered) if hip_filtered else 0, 
                                    len(ankle_deg) if ankle_deg else 0,
                                    len(act_data) if act_data else 0)
                        
                        time_arr = np.array(list(time_data)[-min_len:])
                        hip_arr = np.array(list(hip_data)[-min_len:])
                        
                        # 使用 set_data() 进行高效的增量更新
                        self._plot_lines['hip_raw'].set_data(time_arr, hip_arr)
                        
                        # 更新滤波后的髋角
                        if hip_filtered and len(hip_filtered) > 0 and 'hip_f' in self._plot_lines:
                            valid_indices = [i for i, x in enumerate(hip_filtered[-min_len:]) if x is not None]
                            if len(valid_indices) > 0:
                                valid_time = time_arr[valid_indices]
                                valid_filtered = np.array([hip_filtered[i] for i in valid_indices])
                                self._plot_lines['hip_f'].set_data(valid_time, valid_filtered)
                        
                        # 更新踝关节角度
                        if ankle_deg and len(ankle_deg) > 0 and 'ankle_deg' in self._plot_lines:
                            valid_indices = [i for i, x in enumerate(ankle_deg[-min_len:]) if x is not None]
                            if len(valid_indices) > 0:
                                valid_time = time_arr[valid_indices]
                                valid_ankle = np.array([ankle_deg[i] for i in valid_indices])
                                self._plot_lines['ankle_deg'].set_data(valid_time, valid_ankle)
                        
                        # 更新act逻辑量
                        if act_data and len(act_data) > 0 and 'act' in self._plot_lines:
                            act_mapped = np.array([60.0 if a == 1 else -10.0 for a in act_data[-min_len:]])
                            self._plot_lines['act'].set_data(time_arr, act_mapped)
                        
                        # 自动缩放 Y 轴和 X 轴
                        self.ax1.relim()
                        self.ax1.autoscale_view()
                        
                        self._last_realtime_len = new_len
                        self._last_received_count = self.collector.total_received
            else:
                # 没有数据时显示提示
                # 不再清空已绘制曲线，保持最后一帧可查看
                if not self._plot_initialized:
                    # 仅在从未绘制过时显示提示
                    self.ax1.clear()
                    # ✓ 增强调试信息
                    gait_status = "✓" if self.collector.gait_module_enabled else "✗"
                    hip_status = "✓" if self.collector.hip_module_enabled else "✗"
                    debug_info = (
                        f'等待数据...\n'
                        f'队列大小: {queue_size} | 收到: {self.collector.total_received}\n'
                        f'数据点数: {total_points}\n'
                        f'步态模块: {gait_status} | 髋关节模块: {hip_status}\n'
                        f'[提示] 点击"髋关节数据"或"步态采集"启动数据处理'
                    )
                    self.ax1.text(0.5, 0.5, debug_info, 
                                 horizontalalignment='center', verticalalignment='center',
                                 transform=self.ax1.transAxes, fontsize=11)
                    self.ax1.set_title('实时数据（髋关节原始值、滤波值和踝关节角度）', fontsize=12)
                    self.ax1.set_xlabel('时间 (秒)')
                    self.ax1.set_ylabel('角度 (度)')
                    self.ax1.grid(True)
        
        # 更新第二个图表（步态周期）
        cycle_time, cycle_hip, cycle_ankle = self.collector.get_gait_cycle_data()
        
        if not hasattr(self, '_cycle_plot_initialized'):
            self._cycle_plot_initialized = False
            self._last_cycle_len = 0
            self._cycle_plot_lines = {'hip': None, 'ankle': None}
        
        cycle_data_changed = (len(cycle_time) != self._last_cycle_len)
        
        # 只在数据有实质性变化时重绘
        if cycle_data_changed:
            self.ax2.clear()
            self._cycle_plot_lines = {}
            
            if len(cycle_time) > 0 and len(cycle_hip) > 0 and len(cycle_ankle) > 0:
                # 使用降采样优化性能（如果数据点过多）
                cycle_time_arr = np.array(cycle_time)
                cycle_hip_arr = np.array(cycle_hip)
                cycle_ankle_arr = np.array(cycle_ankle)
                
                # 如果数据点超过 500 个，进行降采样（每 2 个取 1 个）
                if len(cycle_time_arr) > 500:
                    step = max(1, len(cycle_time_arr) // 500)
                    cycle_time_arr = cycle_time_arr[::step]
                    cycle_hip_arr = cycle_hip_arr[::step]
                    cycle_ankle_arr = cycle_ankle_arr[::step]
                
                line_hip, = self.ax2.plot(cycle_time_arr, cycle_hip_arr, 'b-', 
                                         label='髋关节', linewidth=2)
                line_ankle, = self.ax2.plot(cycle_time_arr, cycle_ankle_arr, 'r-', 
                                           label='踝关节', linewidth=2)
                self._cycle_plot_lines['hip'] = line_hip
                self._cycle_plot_lines['ankle'] = line_ankle
                
                self.ax2.set_title('最新步态周期', fontsize=12)
                self.ax2.set_xlabel('时间 (秒)')
                self.ax2.set_ylabel('角度 (度)')
                self.ax2.grid(True, alpha=0.3)
                self.ax2.legend(loc='upper right', fontsize=9)
                self.ax2.relim()
                self.ax2.autoscale()
                
                self._cycle_plot_initialized = True
            else:
                self.ax2.text(0.5, 0.5, '暂无步态周期数据\n（等待数据采集...）', 
                             horizontalalignment='center', verticalalignment='center',
                             transform=self.ax2.transAxes, fontsize=12)
                self.ax2.set_title('最新步态周期', fontsize=12)
                self.ax2.set_xlabel('时间 (秒)')
                self.ax2.set_ylabel('角度 (度)')
                self.ax2.grid(True)
                self._cycle_plot_initialized = True
            
            self._last_cycle_len = len(cycle_time)
        
        # ========== 性能优化2：动态调整刷新频率 ==========
        # 根据数据到达速率动态调整更新间隔，减少不必要的绘制
        if not hasattr(self, '_update_stats'):
            self._update_stats = {'last_check_time': time.time(), 'data_per_sec': 0}
        
        current_time = time.time()
        if current_time - self._update_stats['last_check_time'] >= 1.0:
            # 每秒检查一次数据到达速率
            data_rate = self.collector.total_received - getattr(self, '_last_received_count', 0)
            self._update_stats['data_per_sec'] = data_rate
            self._last_received_count = self.collector.total_received
            self._update_stats['last_check_time'] = current_time
            
            # 根据数据速率调整刷新间隔（500-100Hz的数据自动调整间隔）
            if data_rate > 500:
                # 数据速率高，降低刷新频率以减少卡顿
                self.update_interval = max(50, min(200, 10000 // data_rate))  # 50-200ms
            elif data_rate > 100:
                self.update_interval = 100  # 正常数据速率
            else:
                self.update_interval = 200  # 低速率，不需要频繁刷新
        
        # 仅在必要时绘制（减少 draw() 调用次数）
        # 只有在数据显著更新或周期数据变化时才绘制
        # ✓ 计算数据增量，处理缓冲区循环覆盖的情况
        data_increment = new_len - self._last_realtime_len
        # 当增量为负数时，说明缓冲区被清空或重置，立即触发重绘
        # 当从空白恢复（_last_realtime_len=0 且 new_len>0）时，也要立即重绘
        need_draw = (
            (data_increment > 15 or data_increment < 0) or  # 数据增长或缓冲区重置
            (self._last_realtime_len == 0 and new_len > 0) or  # 从空白状态恢复
            cycle_data_changed
        )
        
        # ✓ 诊断：打印绘制决策
        if not hasattr(self, '_last_draw_log_time'):
            self._last_draw_log_time = time.time()
        current_time = time.time()
        
        # 每2秒打印一次绘制状态
        if current_time - self._last_draw_log_time > 2.0:
            print(f"[update_plots] 绘制决策: need_draw={need_draw}, data增量={data_increment}, cycle_changed={cycle_data_changed}, _plot_initialized={self._plot_initialized}")
            self._last_draw_log_time = current_time
        
        if need_draw:
            self.canvas.draw_idle()  # 使用 draw_idle() 替代 draw()，避免过度刷新
        
        # 定时更新（使用动态间隔）
        self.root.after(self.update_interval, self.update_plots)
    
    def start_serial_monitor(self):
        """启动串口监听（处理原始返回数据并显示到历史记录）"""
        # 处理原始返回数据（记录到历史）
        if self.collector.is_connected():
            raw_lines = self.collector.get_raw_data()
            for line in raw_lines:
                if line.strip():  # 只记录非空行
                    self.add_history(line.strip(), "RX")
        
        # 继续监听
        self.root.after(100, self.start_serial_monitor)  # 每100ms检查一次
    
    def toggle_gait_module(self):
        """切换步态采集模块"""
        if self.collector.gait_module_enabled:
            # 停止步态采集模块
            self.collector.stop_gait_module()
            self.gait_module_btn.config(text="步态采集: 关闭")
            self.add_history("步态采集模块已关闭", "信息")
        else:
            # ✓ 检查其他模块是否已启用
            if self.collector.hip_module_enabled:
                messagebox.showerror("错误", "髋关节数据模块已启用\n请先停止髋关节数据模块再启用步态采集")
                return
            
            # 启动步态采集模块
            if not self.collector.is_connected():
                messagebox.showerror("错误", "请先连接串口")
                return
            
            try:
                self.collector.start_gait_module()
                self.gait_module_btn.config(text="步态采集: 开启")
                self.add_history("步态采集模块已开启", "信息")
            except Exception as e:
                error_msg = f"启动步态采集模块失败: {str(e)}"
                self.add_history(error_msg, "信息")
                messagebox.showerror("错误", str(e))
    
    def toggle_hip_module(self):
        """切换髋关节数据模块"""
        if self.collector.hip_module_enabled:
            # 停止髋关节数据模块
            self.collector.stop_hip_module()
            self.hip_module_btn.config(text="髋关节数据: 关闭")
            self.add_history("髋关节数据模块已关闭", "信息")
        else:
            # ✓ 检查其他模块是否已启用
            if self.collector.gait_module_enabled:
                messagebox.showerror("错误", "步态采集模块已启用\n请先停止步态采集模块再启用髋关节数据模块")
                return
            
            # 启动髋关节数据模块
            if not self.collector.is_connected():
                messagebox.showerror("错误", "请先连接串口")
                return
            
            try:
                self.collector.start_hip_module()
                # ✓ 重置GUI绘制状态，准备新的数据显示
                self._plot_initialized = False
                self._last_realtime_len = 0
                self._plot_lines = {'hip_raw': None, 'hip_f': None, 'ankle_deg': None, 'act': None}
                self.hip_module_btn.config(text="髋关节数据: 开启")
                self.add_history("髋关节数据模块已开启", "信息")
            except Exception as e:
                error_msg = f"启动髋关节数据模块失败: {str(e)}"
                self.add_history(error_msg, "信息")
                messagebox.showerror("错误", str(e))
    
    def toggle_gait_collection(self):
        """切换步态采集（开始/停止）"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "请先连接串口")
            return
        
        if self.gait_collection_enabled:
            # 停止采集
            try:
                self.collector.send_command("gcs")
                self.gait_collection_enabled = False
                self.gait_collection_btn.config(text="开始采集")
                self.add_history("gcs", "TX")
                self.add_history("步态采集已停止", "信息")
            except Exception as e:
                error_msg = f"停止采集失败: {str(e)}"
                self.add_history(error_msg, "信息")
                messagebox.showerror("错误", str(e))
        else:
            # 开始采集
            try:
                self.collector.send_command("gc")
                self.gait_collection_enabled = True
                self.gait_collection_btn.config(text="停止采集")
                self.add_history("gc", "TX")
                self.add_history("步态采集已开始", "信息")
            except Exception as e:
                error_msg = f"开始采集失败: {str(e)}"
                self.add_history(error_msg, "信息")
                messagebox.showerror("错误", str(e))
    
    def send_stand_init(self):
        """发送站立初始化指令（hz 和 az）"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "请先连接串口")
            return
        
        try:
            # 发送髋关节零点指令
            self.collector.send_command("hz")
            self.add_history("hz", "TX")
            
            # 短暂延时后发送踝关节零点指令
            import time
            time.sleep(0.1)
            
            self.collector.send_command("az")
            self.add_history("az", "TX")
            
            self.add_history("站立初始化完成", "信息")
        except Exception as e:
            error_msg = f"站立初始化失败: {str(e)}"
            self.add_history(error_msg, "信息")
            messagebox.showerror("错误", str(e))
    
    def toggle_control_loop(self):
        """切换控制循环启用/禁用"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "请先连接串口")
            return
        
        if self.control_loop_enabled:
            # 禁用控制循环
            try:
                self.collector.send_command("ctrloff")
                self.control_loop_enabled = False
                self.control_loop_btn.config(text="启用控制循环")
                self.add_history("ctrloff", "TX")
                self.add_history("控制循环已禁用", "信息")
            except Exception as e:
                error_msg = f"禁用控制循环失败: {str(e)}"
                self.add_history(error_msg, "信息")
                messagebox.showerror("错误", str(e))
        else:
            # 启用控制循环
            try:
                self.collector.send_command("ctrlon")
                self.control_loop_enabled = True
                self.control_loop_btn.config(text="禁用控制循环")
                self.add_history("ctrlon", "TX")
                self.add_history("控制循环已启用", "信息")
            except Exception as e:
                error_msg = f"启用控制循环失败: {str(e)}"
                self.add_history(error_msg, "信息")
                messagebox.showerror("错误", str(e))

# ============================================================================
# 主程序
# ============================================================================

def main():
    root = tk.Tk()
    app = GaitDataCollectorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
