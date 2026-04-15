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
import re
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

from patient_manager import PatientManager
from training_session import TrainingSession
try:
    from pypinyin import lazy_pinyin, Style
    _PYPINYIN_OK = True
except ImportError:
    _PYPINYIN_OK = False

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
        self.hip_module_enabled = False  # 髋关节数据模块开关
        
        # ========== 后处理模块线程 ==========
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
        
        # 转矩控制阶段：新增数据存储（对应标志位复选框）
        self.ph_data = deque(maxlen=MAX_DATA_POINTS)  # 步态相位 (ph)
        self.s_data = deque(maxlen=MAX_DATA_POINTS)  # 步态进度 (s)
        self.st_data = deque(maxlen=MAX_DATA_POINTS)  # 站立时间 (st)
        self.ank_data = deque(maxlen=MAX_DATA_POINTS)  # 踝关节角度 (ank)
        self.v_data = deque(maxlen=MAX_DATA_POINTS)  # 踝关节速度 (v)
        self.iqT_a_data = deque(maxlen=MAX_DATA_POINTS)  # 踝关节目标力矩 (iqT_a)
        self.iqC_a_data = deque(maxlen=MAX_DATA_POINTS)  # 踝关节实际力矩 (iqC_a)
        self.hip_data_new = deque(maxlen=MAX_DATA_POINTS)  # 髋关节角度 (hip)
        self.hipv_data = deque(maxlen=MAX_DATA_POINTS)  # 髋关节速度 (hipv)
        self.iqT_h_data = deque(maxlen=MAX_DATA_POINTS)  # 髋关节目标力矩 (iqT_h)
        self.iqC_h_data = deque(maxlen=MAX_DATA_POINTS)  # 髋关节实际力矩 (iqC_h)
        self.ph4_data = deque(maxlen=MAX_DATA_POINTS)  # 四相原始值 (ph4: 0/1/2/3)
        self.ph4v_data = deque(maxlen=MAX_DATA_POINTS)  # 四相显示值 (ph4v: 0/10/20/30)
        self.ph4p_data = deque(maxlen=MAX_DATA_POINTS)  # 四相进度 (ph4p)
        self.ph4o_data = deque(maxlen=MAX_DATA_POINTS)  # 四相输出 (ph4o)
        self.ph4d_data = deque(maxlen=MAX_DATA_POINTS)  # 四相退化标志 (ph4d)
        # 状态标志位数据
        self.PF_data = deque(maxlen=MAX_DATA_POINTS)  # 跖屈助力状态 (PF)
        self.DF_data = deque(maxlen=MAX_DATA_POINTS)  # 背屈助力状态 (DF)
        self.UL_data = deque(maxlen=MAX_DATA_POINTS)  # 卸载状态 (UL)
        self.comp_data = deque(maxlen=MAX_DATA_POINTS)  # 退让模式状态 (comp)
        self.cool_data = deque(maxlen=MAX_DATA_POINTS)  # 冷却模式状态 (cool)
        self.abn_data = deque(maxlen=MAX_DATA_POINTS)  # 异常状态 (abn)
        
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
    
    def _parse_torque_format(self, line):
        """
        解析转矩控制阶段的数据格式
        格式：ph=1, s=0.783, st=0.000, ank=-4.49, v=-0.02, iqT_a=0, iqC_a=0, hip=-0.39, hipv=33.21, iqT_h=0, iqC_h=0, flags:PF=0,DF=0,UL=1,comp=0,cool=1,abn=0
        返回：字典格式的数据，兼容现有代码
        """
        try:
            data_dict = {}
            
            # 解析数据部分（flags:之前）
            if 'flags:' in line:
                data_part, flags_part = line.split('flags:', 1)
            else:
                data_part = line
                flags_part = ""
            
            # 解析数据字段
            # 使用正则表达式匹配 key=value 格式
            data_pattern = r'(\w+)=([+-]?\d+\.?\d*)'
            matches = re.findall(data_pattern, data_part)
            
            for key, value in matches:
                try:
                    # 尝试转换为浮点数
                    if '.' in value:
                        data_dict[key] = float(value)
                    else:
                        data_dict[key] = int(value)
                except ValueError:
                    data_dict[key] = value
            
            # 解析flags部分
            if flags_part:
                flags_pattern = r'(\w+)=(\d+)'
                flag_matches = re.findall(flags_pattern, flags_part)
                for flag_key, flag_value in flag_matches:
                    try:
                        data_dict[flag_key] = int(flag_value)
                    except ValueError:
                        data_dict[flag_key] = flag_value
            
            # 添加时间戳（如果没有，使用当前时间）
            if 't' not in data_dict:
                data_dict['t'] = int(time.time() * 1000)  # 毫秒时间戳
            
            # 兼容性：将新字段映射到旧字段（如果存在）
            if 'ank' in data_dict and 'a' not in data_dict:
                data_dict['a'] = data_dict['ank']
            if 'hip' in data_dict and 'h' not in data_dict:
                data_dict['h'] = data_dict['hip']
            if 'ph' in data_dict and 'phase' not in data_dict:
                data_dict['phase'] = data_dict['ph']
            if 's' in data_dict and 'swing_progress' not in data_dict:
                data_dict['swing_progress'] = data_dict['s']
            
            return data_dict if len(data_dict) > 0 else None
            
        except Exception as e:
            # 解析失败，返回None
            return None
    
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
                    # 仅过滤命令回显（如 "> Command: xxx"），保留 ">>> ..." 系统响应
                    if line and not line.startswith('> Command:') and not line.startswith('Command:'):
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
    
    def start_hip_module(self):
        """启动髋关节数据模块"""
        if self.hip_module_enabled:
            return False  # 已经在运行
        
        # ✓ 清空旧缓冲区，确保第二次启动时能正确更新
        # 同时清空队列积压，避免曲线延迟显示旧数据
        self.clear_runtime_queues()
        self.time_data.clear()
        self.hip_data.clear()
        self.hip_filtered_data.clear()
        self.hip_velocity_data.clear()
        self.hip_velocity_filtered_data.clear()
        self.phase_data.clear()
        self.swing_progress_data.clear()

        # 清空新格式数据缓冲（复选框对应字段）
        self.ph_data.clear()
        self.s_data.clear()
        self.st_data.clear()
        self.ank_data.clear()
        self.v_data.clear()
        self.iqT_a_data.clear()
        self.iqC_a_data.clear()
        self.hip_data_new.clear()
        self.hipv_data.clear()
        self.iqT_h_data.clear()
        self.iqC_h_data.clear()
        self.ph4_data.clear()
        self.ph4v_data.clear()
        self.ph4p_data.clear()
        self.ph4o_data.clear()
        self.ph4d_data.clear()
        self.PF_data.clear()
        self.DF_data.clear()
        self.UL_data.clear()
        self.comp_data.clear()
        self.cool_data.clear()
        self.abn_data.clear()
        
        self.hip_module_enabled = True
        # 启动髋关节数据处理线程
        if not self.hip_process_thread or not self.hip_process_thread.is_alive():
            self.hip_process_thread = threading.Thread(target=self._hip_process_loop, daemon=True)
            self.hip_process_thread.start()
        
        return True
    
    def stop_hip_module(self):
        """停止髋关节数据模块"""
        self.hip_module_enabled = False
        if self.hip_process_thread:
            self.hip_process_thread.join(timeout=2)
        
        # ✓ 不再清空缓冲区，保留数据以便用户保存
        # 数据会保留在缓冲区中，直到用户重新启动采集或手动清空
    
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
                        
                        # 跳过命令回显（但保留 ">>> ..." 系统响应）
                        if line.startswith('> Command:') or line.startswith('Command:'):
                            continue
                        
                        # 记录所有原始返回数据到队列（用于历史记录）
                        self.raw_data_queue.put(original_line)
                        
                        # 统一仅解析 JSON 数据（sendGaitData 固定 schema）
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
                                        
                                        # ✓ 定期更新诊断时间戳
                                        current_time = time.time()
                                        if current_time - last_diagnostic_time > 1.0:
                                            last_diagnostic_time = current_time
                                    else:
                                        # ✓ 跳过缺失关键字段的数据
                                        continue
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
    
    def _hip_process_loop(self):
        """髋关节数据模块处理循环（独立线程）"""
        data_count = 0
        while self.hip_module_enabled:
            try:
                if not self.data_queue.empty():
                    data = self.data_queue.get(timeout=0.1)
                    
                    # 统一按 sendGaitData 的 JSON 键处理：
                    # 缺失字段保持为 None，绘图阶段按复选框与有效值决定是否显示
                    timestamp = data.get('t', None)  # 毫秒
                    hip_raw = data.get('h', None)    # 髋关节原始角度

                    # 至少需要时间戳和髋角
                    if hip_raw is None or timestamp is None:
                        continue

                    hip_f = data.get('hf', hip_raw)  # 未上传 hf 时回退 h
                    hip_vel_f = data.get('hipv') if data.get('hipv') is not None else data.get('hvf', None)
                    phase = data.get('ph') if data.get('ph') is not None else data.get('phase', 0)
                    swing_progress = data.get('s', 0.0)
                    ankle_ref = data.get('ar', None)

                    # 基础缓冲区（用于状态显示/hip_f 主线）
                    self.time_data.append(timestamp)
                    self.hip_filtered_data.append(hip_f if hip_f is not None else None)
                    self.hip_velocity_filtered_data.append(hip_vel_f if hip_vel_f is not None else None)
                    self.phase_data.append(phase)
                    self.swing_progress_data.append(swing_progress)
                    self.ankle_ref_data.append(ankle_ref if ankle_ref is not None else None)

                    # 复选框数据通道：字段缺失即 None（不显示）
                    self.ph_data.append(data.get('ph'))
                    self.s_data.append(data.get('s'))
                    self.st_data.append(data.get('st'))
                    self.ank_data.append(data.get('ank'))
                    self.v_data.append(data.get('v'))
                    self.iqT_a_data.append(data.get('iqT_a'))
                    self.iqC_a_data.append(data.get('iqC_a'))
                    self.hip_data_new.append(data.get('hip'))
                    self.hipv_data.append(data.get('hipv'))
                    self.iqT_h_data.append(data.get('iqT_h'))
                    self.iqC_h_data.append(data.get('iqC_h'))
                    self.ph4_data.append(data.get('ph4'))
                    self.ph4v_data.append(data.get('ph4v'))
                    self.ph4p_data.append(data.get('ph4p'))
                    self.ph4o_data.append(data.get('ph4o'))
                    self.ph4d_data.append(data.get('ph4d'))
                    self.PF_data.append(data.get('PF'))
                    self.DF_data.append(data.get('DF'))
                    self.UL_data.append(data.get('UL'))
                    self.comp_data.append(data.get('comp'))
                    self.cool_data.append(data.get('cool'))
                    self.abn_data.append(data.get('abn'))

                    # 保证各复选框队列长度与 time_data 一致
                    time_len = len(self.time_data)
                    torque_queues = [
                        self.ph_data, self.s_data, self.st_data, self.ank_data, self.v_data,
                        self.iqT_a_data, self.iqC_a_data, self.hip_data_new, self.hipv_data,
                        self.iqT_h_data, self.iqC_h_data, self.ph4_data, self.ph4v_data,
                        self.ph4p_data, self.ph4o_data, self.ph4d_data, self.PF_data, self.DF_data,
                        self.UL_data, self.comp_data, self.cool_data, self.abn_data
                    ]
                    for q in torque_queues:
                        while len(q) < time_len:
                            q.append(None)
                    
                    data_count += 1
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
        # 清空队列积压，避免清除后仍显示历史数据
        self.clear_runtime_queues()

        # 清除实时数据
        self.time_data.clear()
        self.hip_data.clear()
        self.ankle_data.clear()
        self.hip_filtered_data.clear()
        self.hip_velocity_data.clear()
        self.hip_velocity_filtered_data.clear()
        self.phase_data.clear()
        self.swing_progress_data.clear()

        # 清空新格式数据缓冲（复选框对应字段）
        self.ph_data.clear()
        self.s_data.clear()
        self.st_data.clear()
        self.ank_data.clear()
        self.v_data.clear()
        self.iqT_a_data.clear()
        self.iqC_a_data.clear()
        self.hip_data_new.clear()
        self.hipv_data.clear()
        self.iqT_h_data.clear()
        self.iqC_h_data.clear()
        self.ph4_data.clear()
        self.ph4v_data.clear()
        self.ph4p_data.clear()
        self.ph4o_data.clear()
        self.ph4d_data.clear()
        self.PF_data.clear()
        self.DF_data.clear()
        self.UL_data.clear()
        self.comp_data.clear()
        self.cool_data.clear()
        self.abn_data.clear()
        
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

    def _drain_queue(self, q: queue.Queue, max_items: int = 100000):
        """清空队列，避免积压旧数据导致曲线滞后。"""
        drained = 0
        while drained < max_items:
            try:
                q.get_nowait()
                drained += 1
            except queue.Empty:
                break
        return drained

    def clear_runtime_queues(self):
        """清空运行时队列（串口解析后的数据与原始行）。"""
        self._drain_queue(self.data_queue)
        self._drain_queue(self.raw_data_queue)
    
    def get_realtime_data(self):
        """获取实时数据（用于绘图，并进行性能优化的数据降采样）"""
        # 仅返回绘图仍需要的数据：time + hip_f（滤波髋角）
        if len(self.time_data) == 0 or len(self.hip_filtered_data) == 0:
            return [], []
        
        # 确保数据长度一致（取最小长度）
        min_len = min(len(self.time_data), len(self.hip_filtered_data))
        if min_len == 0:
            return [], []
        
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
        hip_filtered = [self.hip_filtered_data[i] if i < len(self.hip_filtered_data) else None for i in indices]
        
        return relative_time, hip_filtered
    
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
# 拼音过滤下拉组件
# ============================================================================

def _get_pinyin_initials(text: str) -> str:
    """返回汉字字符串的声母首字母缩写（如 '张三' → 'zs'）。需要 pypinyin。"""
    if not _PYPINYIN_OK:
        return ""
    initials = lazy_pinyin(text, style=Style.FIRST_LETTER)
    return "".join(initials).lower()


def _get_full_pinyin(text: str) -> str:
    """返回汉字字符串的全拼（如 '张三' → 'zhangsan'）。需要 pypinyin。"""
    if not _PYPINYIN_OK:
        return ""
    full = lazy_pinyin(text, style=Style.NORMAL)
    return "".join(full).lower()


class PinyinFilterCombo(ttk.Frame):
    """
    支持拼音筛选的自定义下拉组合框。
    items 格式：[{"label": str, "data": any}, ...]
    用户可通过汉字、全拼或首字母缩写过滤列表。
    """

    POPUP_MAX_HEIGHT = 12  # Listbox 最多显示行数

    def __init__(self, parent, width: int = 18, on_select=None, **kwargs):
        super().__init__(parent, **kwargs)
        self._all_items: list = []
        self._filtered_items: list = []
        self._on_select = on_select
        self._popup: tk.Toplevel | None = None
        self._listbox: tk.Listbox | None = None
        self._suppress_trace = False

        self._text_var = tk.StringVar()
        self._entry = ttk.Entry(self, textvariable=self._text_var, width=width)
        self._entry.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self._arrow_btn = ttk.Button(self, text="▼", width=2, command=self._toggle_popup)
        self._arrow_btn.pack(side=tk.LEFT)

        self._text_var.trace_add("write", self._on_text_changed)
        self._entry.bind("<FocusIn>", lambda e: self._show_popup())
        self._entry.bind("<Return>", self._on_enter)
        self._entry.bind("<Escape>", lambda e: self._hide_popup())
        self._entry.bind("<Down>", self._focus_list)

    # -- 公共接口 --

    def set_items(self, items: list):
        """设置候选列表。items: [{"label": str, "data": any}]"""
        self._all_items = items
        # 预计算拼音索引
        for item in self._all_items:
            label = item["label"]
            item["_initials"] = _get_pinyin_initials(label)
            item["_full_pinyin"] = _get_full_pinyin(label)
        self._filtered_items = list(self._all_items)

    def set_text(self, text: str):
        """程序化设置显示文字（不触发筛选弹出）。"""
        self._suppress_trace = True
        self._text_var.set(text)
        self._suppress_trace = False

    def get_text(self) -> str:
        return self._text_var.get()

    # -- 内部逻辑 --

    def _on_text_changed(self, *_):
        if self._suppress_trace:
            return
        self._update_filter()
        self._show_popup()

    def _update_filter(self):
        query = self._text_var.get().strip().lower()
        if not query:
            self._filtered_items = list(self._all_items)
        else:
            result = []
            for item in self._all_items:
                label_lower = item["label"].lower()
                initials = item.get("_initials", "")
                full_py = item.get("_full_pinyin", "")
                if (query in label_lower or
                        initials.startswith(query) or
                        full_py.startswith(query)):
                    result.append(item)
            self._filtered_items = result
        if self._listbox:
            self._populate_listbox()

    def _toggle_popup(self):
        if self._popup and self._popup.winfo_exists():
            self._hide_popup()
        else:
            self._update_filter()
            self._show_popup()

    def _show_popup(self):
        if not self._filtered_items:
            self._hide_popup()
            return
        if self._popup and self._popup.winfo_exists():
            self._populate_listbox()
            return

        # 计算弹出位置（Entry 正下方）
        self.update_idletasks()
        x = self._entry.winfo_rootx()
        y = self._entry.winfo_rooty() + self._entry.winfo_height()
        width = self._entry.winfo_width() + self._arrow_btn.winfo_width()

        popup = tk.Toplevel(self)
        popup.wm_overrideredirect(True)
        popup.wm_geometry(f"{width}x1+{x}+{y}")
        popup.lift()
        self._popup = popup

        sb = ttk.Scrollbar(popup, orient=tk.VERTICAL)
        lb = tk.Listbox(popup, yscrollcommand=sb.set, selectmode=tk.SINGLE,
                        font=("", 9), relief=tk.SOLID, bd=1)
        sb.config(command=lb.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        lb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._listbox = lb

        lb.bind("<<ListboxSelect>>", self._on_listbox_select)
        lb.bind("<Return>", self._on_listbox_return)
        lb.bind("<Escape>", lambda e: self._hide_popup())

        self._populate_listbox()

        # 点击弹窗外关闭
        popup.bind("<FocusOut>", self._on_popup_focus_out)

    def _populate_listbox(self):
        if self._listbox is None:
            return
        self._listbox.delete(0, tk.END)
        for item in self._filtered_items:
            self._listbox.insert(tk.END, item["label"])
        rows = min(len(self._filtered_items), self.POPUP_MAX_HEIGHT)
        # 动态调整高度（每行约 18px）
        if self._popup and self._popup.winfo_exists():
            self.update_idletasks()
            x = self._entry.winfo_rootx()
            y = self._entry.winfo_rooty() + self._entry.winfo_height()
            width = self._entry.winfo_width() + self._arrow_btn.winfo_width()
            height = rows * 18 + 4
            self._popup.wm_geometry(f"{width}x{height}+{x}+{y}")

    def _hide_popup(self):
        if self._popup and self._popup.winfo_exists():
            self._popup.destroy()
        self._popup = None
        self._listbox = None

    def _on_popup_focus_out(self, event):
        # 延迟一帧判断焦点是否真的离开
        self.after(50, self._check_focus_and_hide)

    def _check_focus_and_hide(self):
        focused = self.focus_get()
        if focused not in (self._entry, self._listbox, self._arrow_btn):
            self._hide_popup()

    def _on_listbox_select(self, event):
        lb = self._listbox
        if lb is None:
            return
        sel = lb.curselection()
        if sel:
            idx = sel[0]
            if idx < len(self._filtered_items):
                self._select_item(self._filtered_items[idx])

    def _on_listbox_return(self, event):
        self._on_listbox_select(event)

    def _on_enter(self, event):
        if self._filtered_items:
            self._select_item(self._filtered_items[0])

    def _select_item(self, item: dict):
        self._suppress_trace = True
        self._text_var.set(item["label"])
        self._suppress_trace = False
        self._hide_popup()
        if self._on_select:
            self._on_select(item)

    def _focus_list(self, event):
        if self._listbox and self._listbox.winfo_exists():
            self._listbox.focus_set()
            if self._listbox.size() > 0:
                self._listbox.selection_set(0)
                self._listbox.see(0)


# ============================================================================
# GUI主窗口
# ============================================================================

class GaitDataCollectorGUI:
    def __init__(self, root):
        # 初始化实时曲线长度缓存（用于性能优化）
        self._last_realtime_len = -1  # 最近一次实时曲线点数，用于避免重复重绘
        self._last_realtime_latest = None  # 最近一次实时曲线的最新时间戳，用于滑动窗口检测
        self.root = root
        self.root.title("步态数据采集与可视化")
        self.root.geometry("1200x800")
        
        self.collector = GaitDataCollector()
        self.update_interval = 50  # 更新间隔（毫秒）
        self.history_max_lines = 500  # 历史记录最大行数

        # 十字准星（鼠标悬停显示）
        self._crosshair_vline = None
        self._crosshair_hline = None
        
        # 按钮状态跟踪
        self.control_loop_enabled = False  # 控制循环启用状态

        # 患者管理
        self.patient_manager = PatientManager()
        self.current_patient: dict | None = None
        self.training_session = TrainingSession()
        self._last_flush_check: float = 0.0  # 上次临时文件刷新检查时间

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

    # =========================================================================
    # 患者管理面板
    # =========================================================================

    def _build_patient_panel(self, parent: ttk.LabelFrame):
        """在 parent 内构建患者管理 UI。"""
        parent.columnconfigure(1, weight=1)

        # ── 第0行：患者选择 ──
        ttk.Label(parent, text="患者:").grid(row=0, column=0, sticky=tk.W, pady=3)

        # 拼音过滤组合框（自定义）
        self._patient_pinyin_combo = PinyinFilterCombo(parent, width=18,
                                                        on_select=self._on_patient_selected)
        self._patient_pinyin_combo.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=4, pady=3)

        btn_row = ttk.Frame(parent)
        btn_row.grid(row=0, column=2, sticky=tk.W, padx=(2, 0))
        ttk.Button(btn_row, text="新建患者", width=8,
                   command=self._open_new_patient_dialog).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_row, text="查看记录", width=8,
                   command=self._open_record_viewer).pack(side=tk.LEFT, padx=2)

        # ── 第1行：当前患者信息 ──
        self._patient_info_var = tk.StringVar(value="未选择患者")
        ttk.Label(parent, textvariable=self._patient_info_var,
                  foreground="#555555").grid(row=1, column=0, columnspan=3,
                                             sticky=tk.W, pady=(0, 4))

        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(
            row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=4)

        # ── 第3行：训练类型 ──
        ttk.Label(parent, text="训练类型:").grid(row=3, column=0, sticky=tk.W, pady=3)
        self._training_type_var = tk.StringVar(value="active")
        type_frame = ttk.Frame(parent)
        type_frame.grid(row=3, column=1, columnspan=2, sticky=tk.W)
        ttk.Radiobutton(type_frame, text="主动训练", variable=self._training_type_var,
                        value="active").pack(side=tk.LEFT, padx=(0, 10))
        self._passive_radio = ttk.Radiobutton(type_frame, text="被动训练（开发中）",
                                               variable=self._training_type_var,
                                               value="passive", state=tk.DISABLED)
        self._passive_radio.pack(side=tk.LEFT)

        # ── 第4行：训练控制按钮 ──
        ctrl_frame = ttk.Frame(parent)
        ctrl_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=4)
        self._start_btn = ttk.Button(ctrl_frame, text="开始训练", width=9,
                                      command=self._on_start_training)
        self._start_btn.pack(side=tk.LEFT, padx=2)
        self._pause_btn = ttk.Button(ctrl_frame, text="暂停", width=7,
                                      command=self._on_pause_resume, state=tk.DISABLED)
        self._pause_btn.pack(side=tk.LEFT, padx=2)
        self._stop_btn = ttk.Button(ctrl_frame, text="结束训练", width=9,
                                     command=self._on_stop_training, state=tk.DISABLED)
        self._stop_btn.pack(side=tk.LEFT, padx=2)

        # ── 第5行：训练统计 ──
        stats_frame = ttk.Frame(parent)
        stats_frame.grid(row=5, column=0, columnspan=3, sticky=tk.W, pady=(2, 0))
        ttk.Label(stats_frame, text="时长:").pack(side=tk.LEFT)
        self._duration_var = tk.StringVar(value="00:00:00")
        ttk.Label(stats_frame, textvariable=self._duration_var,
                  font=("Consolas", 11, "bold"), foreground="#0055AA",
                  width=9).pack(side=tk.LEFT, padx=(2, 12))
        ttk.Label(stats_frame, text="步数:").pack(side=tk.LEFT)
        self._steps_var = tk.StringVar(value="0 步")
        ttk.Label(stats_frame, textvariable=self._steps_var,
                  font=("Consolas", 11, "bold"), foreground="#AA5500",
                  width=7).pack(side=tk.LEFT, padx=(2, 0))

        # 初始刷新患者列表
        self._refresh_patient_list()

    def _refresh_patient_list(self):
        """重新加载患者列表到拼音过滤组合框。"""
        patients = self.patient_manager.get_all_patients()
        # 每个条目格式：{id:编号  name:姓名} 显示为 "001 张三"
        items = [{"label": f"{p['id']} {p['name']}", "data": p} for p in patients]
        self._patient_pinyin_combo.set_items(items)

    def _on_patient_selected(self, item: dict):
        """用户从下拉列表选择患者后的回调。"""
        patient = item["data"]
        self.current_patient = patient
        age = PatientManager.calc_age(patient.get("birth_date", ""))
        h = patient.get("height", "")
        w = patient.get("weight", "")
        self._patient_info_var.set(
            f"编号: {patient['id']}  姓名: {patient['name']}  "
            f"年龄: {age}岁  身高: {h}cm  体重: {w}kg"
        )

    def _update_training_stats_ui(self):
        """刷新训练时长和步数标签。"""
        stats = self.training_session.get_stats()
        self._duration_var.set(TrainingSession.format_duration(stats["active_duration_s"]))
        self._steps_var.set(f"{stats['step_count']} 步")

    # =========================================================================
    # 训练控制
    # =========================================================================

    def _on_start_training(self):
        """开始训练按钮处理。"""
        if self.current_patient is None:
            messagebox.showwarning("提示", "请先选择患者")
            return
        if not self.collector.is_connected():
            messagebox.showwarning("提示", "请先连接串口设备")
            return
        if self.training_session.state != TrainingSession.STATE_IDLE:
            messagebox.showwarning("提示", "当前已有训练会话，请先结束")
            return

        training_type = self._training_type_var.get()
        patient_dir = self.patient_manager.get_patient_dir(
            self.current_patient["id"], self.current_patient.get("name", ""))

        try:
            self.training_session.start(self.current_patient, training_type, patient_dir)
        except Exception as e:
            messagebox.showerror("错误", f"启动训练失败: {e}")
            return

        # 主动训练：启动下位机数据上传
        if training_type == "active":
            if not self.collector.hip_module_enabled:
                try:
                    self.collector.start_hip_module()
                    self.collector.send_command("ctrlon")
                    self.add_history("ctrlon", "TX")
                    self.control_loop_enabled = True
                    self.control_loop_btn.config(text="禁用控制循环")
                    self.collector.send_command("gc")
                    self.add_history("gc", "TX")
                    self._plot_initialized = False
                    self._last_realtime_len = 0
                    self._plot_lines = {'hip_f': None}
                    self.hip_module_btn.config(text="髋关节数据: 开启")
                except Exception as e:
                    self.add_history(f"启动数据采集失败: {e}", "信息")

        self._start_btn.config(state=tk.DISABLED)
        self._pause_btn.config(state=tk.NORMAL, text="暂停")
        self._stop_btn.config(state=tk.NORMAL)
        self.add_history(f"训练开始: {self.current_patient['name']} / "
                         f"{'主动' if training_type == 'active' else '被动'}训练", "信息")

    def _on_pause_resume(self):
        """暂停/恢复按钮处理。"""
        state = self.training_session.state
        if state == TrainingSession.STATE_RUNNING:
            self.training_session.pause()
            try:
                self.collector.send_command("ctrloff")
                self.add_history("ctrloff", "TX")
            except Exception:
                pass
            self._pause_btn.config(text="恢复")
            self.add_history("训练已暂停", "信息")
        elif state == TrainingSession.STATE_PAUSED:
            self.training_session.resume()
            try:
                self.collector.send_command("ctrlon")
                self.add_history("ctrlon", "TX")
            except Exception:
                pass
            self._pause_btn.config(text="暂停")
            self.add_history("训练已恢复", "信息")

    def _on_stop_training(self):
        """结束训练按钮处理。"""
        if self.training_session.state not in (
                TrainingSession.STATE_RUNNING, TrainingSession.STATE_PAUSED):
            return
        if not messagebox.askyesno("确认", "确定结束本次训练并保存数据？"):
            return

        try:
            saved_path = self.training_session.stop()
        except Exception as e:
            messagebox.showerror("错误", f"保存训练数据失败: {e}")
            return

        # 停止下位机数据上传
        try:
            self.collector.send_command("gcs")
            self.add_history("gcs", "TX")
        except Exception:
            pass

        self.training_session.reset()
        self._start_btn.config(state=tk.NORMAL)
        self._pause_btn.config(state=tk.DISABLED, text="暂停")
        self._stop_btn.config(state=tk.DISABLED)
        self._duration_var.set("00:00:00")
        self._steps_var.set("0 步")

        self.add_history(f"训练已结束，文件已保存", "信息")
        messagebox.showinfo("训练完成", f"训练数据已保存至:\n{saved_path}")

    # =========================================================================
    # 新建患者对话框
    # =========================================================================

    def _open_new_patient_dialog(self):
        """打开新建患者的模态对话框。"""
        dialog = tk.Toplevel(self.root)
        dialog.title("新建患者")
        dialog.resizable(False, False)
        dialog.grab_set()  # 模态

        # 居中显示
        dialog.update_idletasks()
        self.root.update_idletasks()
        x = self.root.winfo_x() + self.root.winfo_width() // 2 - 180
        y = self.root.winfo_y() + self.root.winfo_height() // 2 - 160
        dialog.geometry(f"360x320+{x}+{y}")

        pad = {"padx": 12, "pady": 5}
        frame = ttk.Frame(dialog, padding="16")
        frame.pack(fill=tk.BOTH, expand=True)
        frame.columnconfigure(1, weight=1)

        # 编号（只读）
        new_id = self.patient_manager.generate_id()
        ttk.Label(frame, text="患者编号:").grid(row=0, column=0, sticky=tk.W, **pad)
        ttk.Label(frame, text=new_id, font=("", 10, "bold"),
                  foreground="#0055AA").grid(row=0, column=1, sticky=tk.W, **pad)

        # 姓名
        ttk.Label(frame, text="姓名 *:").grid(row=1, column=0, sticky=tk.W, **pad)
        name_var = tk.StringVar()
        ttk.Entry(frame, textvariable=name_var, width=22).grid(
            row=1, column=1, sticky=(tk.W, tk.E), **pad)

        # 出生日期
        ttk.Label(frame, text="出生日期 *:").grid(row=2, column=0, sticky=tk.W, **pad)
        birth_var = tk.StringVar(value="1980-01-01")
        birth_entry = ttk.Entry(frame, textvariable=birth_var, width=22)
        birth_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), **pad)
        ttk.Label(frame, text="格式 YYYY-MM-DD", foreground="#888888",
                  font=("", 8)).grid(row=3, column=1, sticky=tk.W, padx=12, pady=0)

        # 身高
        ttk.Label(frame, text="身高 (cm):").grid(row=4, column=0, sticky=tk.W, **pad)
        height_var = tk.StringVar()
        ttk.Entry(frame, textvariable=height_var, width=22).grid(
            row=4, column=1, sticky=(tk.W, tk.E), **pad)

        # 体重
        ttk.Label(frame, text="体重 (kg):").grid(row=5, column=0, sticky=tk.W, **pad)
        weight_var = tk.StringVar()
        ttk.Entry(frame, textvariable=weight_var, width=22).grid(
            row=5, column=1, sticky=(tk.W, tk.E), **pad)

        def _save():
            name = name_var.get().strip()
            birth = birth_var.get().strip()
            if not name:
                messagebox.showwarning("提示", "请输入患者姓名", parent=dialog)
                return
            if not birth or len(birth) != 10:
                messagebox.showwarning("提示", "请输入有效的出生日期（YYYY-MM-DD）",
                                       parent=dialog)
                return
            try:
                h = float(height_var.get()) if height_var.get().strip() else 0.0
                w = float(weight_var.get()) if weight_var.get().strip() else 0.0
            except ValueError:
                messagebox.showwarning("提示", "身高/体重请输入数字", parent=dialog)
                return
            try:
                patient = self.patient_manager.create_patient(name, birth, h, w)
            except Exception as e:
                messagebox.showerror("错误", f"创建患者失败: {e}", parent=dialog)
                return
            self._refresh_patient_list()
            # 自动选中新建患者
            self._patient_pinyin_combo.set_text(f"{patient['id']} {patient['name']}")
            self.current_patient = patient
            self._on_patient_selected({"label": f"{patient['id']} {patient['name']}",
                                       "data": patient})
            dialog.destroy()
            messagebox.showinfo("成功", f"患者【{name}】已创建，编号: {patient['id']}")

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=6, column=0, columnspan=2, pady=12)
        ttk.Button(btn_frame, text="保存", command=_save, width=10).pack(side=tk.LEFT, padx=8)
        ttk.Button(btn_frame, text="取消", command=dialog.destroy, width=10).pack(
            side=tk.LEFT, padx=8)

        name_var.trace_add("write", lambda *_: None)
        frame.nametowidget(frame.grid_slaves(row=1, column=1)[0]).focus_set()

    # =========================================================================
    # 查看训练记录对话框
    # =========================================================================

    def _open_record_viewer(self):
        """打开查看患者训练记录的模态对话框。"""
        if self.current_patient is None:
            messagebox.showwarning("提示", "请先选择患者")
            return

        records = self.patient_manager.get_training_records(self.current_patient)

        dialog = tk.Toplevel(self.root)
        dialog.title(f"{self.current_patient['name']} 的训练记录")
        dialog.grab_set()
        dialog.update_idletasks()
        x = self.root.winfo_x() + 60
        y = self.root.winfo_y() + 60
        dialog.geometry(f"820x520+{x}+{y}")

        # 左侧：记录列表
        left = ttk.Frame(dialog, padding="8")
        left.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(left, text="训练记录列表", font=("", 10, "bold")).pack(anchor=tk.W)

        list_frame = ttk.Frame(left)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=6)
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL)
        record_listbox = tk.Listbox(list_frame, width=36, yscrollcommand=scrollbar.set,
                                    selectmode=tk.SINGLE, font=("", 9))
        scrollbar.config(command=record_listbox.yview)
        record_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 右侧：详情面板
        right = ttk.Frame(dialog, padding="8")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Label(right, text="记录详情", font=("", 10, "bold")).pack(anchor=tk.W)

        detail_text = tk.Text(right, height=8, width=44, wrap=tk.WORD, state=tk.DISABLED,
                              font=("", 9), relief=tk.SUNKEN, bd=1)
        detail_text.pack(fill=tk.X, pady=(4, 8))

        # 内嵌小图（用于预览）
        preview_fig = Figure(figsize=(5.5, 3.5), dpi=85)
        preview_ax = preview_fig.add_subplot(1, 1, 1)
        preview_canvas = FigureCanvasTkAgg(preview_fig, right)
        preview_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        btn_row = ttk.Frame(right)
        btn_row.pack(fill=tk.X, pady=6)

        _current_record = {"data": None, "path": None}

        def _show_record(filepath):
            """加载并展示选中记录。"""
            try:
                rec = self.patient_manager.load_training_record(filepath)
            except Exception as e:
                messagebox.showerror("错误", f"读取记录失败: {e}", parent=dialog)
                return
            _current_record["data"] = rec
            _current_record["path"] = filepath

            # 更新文字详情
            training_type = "主动训练" if rec.get("training_type") == "active" else "被动训练"
            active_s = rec.get("active_duration_s", 0)
            m, s = divmod(int(active_s), 60)
            h, m = divmod(m, 60)
            pauses = rec.get("pauses", [])
            detail = (
                f"训练类型：{training_type}\n"
                f"开始时间：{rec.get('start_time', '')}\n"
                f"结束时间：{rec.get('end_time', '')}\n"
                f"有效时长：{h:02d}:{m:02d}:{s:02d}\n"
                f"步数：{rec.get('step_count', 0)} 步\n"
                f"暂停次数：{len(pauses)} 次\n"
                f"数据点数：{len(rec.get('realtime_data', []))} 条"
            )
            detail_text.config(state=tk.NORMAL)
            detail_text.delete("1.0", tk.END)
            detail_text.insert(tk.END, detail)
            detail_text.config(state=tk.DISABLED)

            # 绘制预览曲线
            preview_ax.clear()
            rt = rec.get("realtime_data", [])
            if rt:
                times = [d.get("t", i * 50) for i, d in enumerate(rt)]
                t0 = times[0]
                times_s = [(t - t0) / 1000.0 for t in times]
                ank_vals = [d.get("ank") for d in rt]
                hip_vals = [d.get("hip") for d in rt]

                def _filter_none(xs, ys):
                    pairs = [(x, y) for x, y in zip(xs, ys) if y is not None]
                    return ([p[0] for p in pairs], [p[1] for p in pairs]) if pairs else ([], [])

                tx, ax_v = _filter_none(times_s, ank_vals)
                th, hi_v = _filter_none(times_s, hip_vals)
                if tx:
                    preview_ax.plot(tx, ax_v, color="#E06000", linewidth=1.2,
                                    label="踝关节(ank)")
                if th:
                    preview_ax.plot(th, hi_v, color="#0066CC", linewidth=1.2,
                                    label="髋关节(hip)")
                preview_ax.set_xlabel("时间 (s)", fontsize=8)
                preview_ax.set_ylabel("角度 (°)", fontsize=8)
                preview_ax.legend(fontsize=7)
                preview_ax.grid(True, alpha=0.3)
            preview_fig.tight_layout()
            preview_canvas.draw_idle()

        def _on_list_select(event):
            sel = record_listbox.curselection()
            if not sel:
                return
            idx = sel[0]
            if idx < len(records):
                _show_record(records[idx])

        def _load_to_main():
            """将选中记录加载到主界面图表。"""
            rec = _current_record["data"]
            if rec is None:
                messagebox.showwarning("提示", "请先选择一条记录", parent=dialog)
                return
            rt = rec.get("realtime_data", [])
            if not rt:
                messagebox.showwarning("提示", "该记录没有实时数据", parent=dialog)
                return
            # 填充 collector 的数据缓冲，复用已有的绘图机制
            self.collector.clear_all_data()
            self.collector.start_hip_module()
            for d in rt:
                self.collector.data_queue.put(d)
            # 给处理线程一点时间消化队列
            self.root.after(300, lambda: None)
            self._plot_initialized = False
            self._last_realtime_len = 0
            self._plot_lines = {"hip_f": None}
            self.hip_module_btn.config(text="髋关节数据: 开启")
            self.add_history(f"已载入训练记录: {os.path.basename(_current_record['path'])}", "信息")
            dialog.destroy()

        record_listbox.bind("<<ListboxSelect>>", _on_list_select)
        record_listbox.bind("<Double-Button-1>", lambda e: _load_to_main())

        ttk.Button(btn_row, text="加载到主界面", command=_load_to_main,
                   width=14).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="关闭", command=dialog.destroy, width=8).pack(
            side=tk.LEFT, padx=4)

        # 填充列表
        if not records:
            record_listbox.insert(tk.END, "（暂无训练记录）")
        else:
            for fp in records:
                try:
                    rec = self.patient_manager.load_training_record(fp)
                    label = PatientManager.format_record_label(fp, rec)
                except Exception:
                    label = os.path.basename(fp)
                record_listbox.insert(tk.END, label)
            # 默认选中第一条
            record_listbox.selection_set(0)
            _show_record(records[0])

    # =========================================================================
    # 拼音过滤组合框（内部组件类）
    # =========================================================================

    def create_widgets(self):
        """创建GUI组件"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # 左列：患者管理（上）+ 控制面板（下）
        main_frame.columnconfigure(0, weight=0)  # 左列不扩展
        main_frame.columnconfigure(1, weight=1)  # 右列（图表）扩展
        main_frame.rowconfigure(0, weight=0)     # 患者管理行不扩展
        main_frame.rowconfigure(1, weight=1)     # 控制面板行扩展

        # ── 患者管理面板（左上角）──
        patient_frame = ttk.LabelFrame(main_frame, text="患者管理", padding="8")
        patient_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10), pady=(0, 6))
        self._build_patient_panel(patient_frame)

        # 左侧控制面板（移至 row=1）
        control_frame = ttk.LabelFrame(main_frame, text="控制面板", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
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
        
        # ── 可折叠控制区域（默认折叠）──
        self._ctrl_expanded = False

        def _toggle_ctrl_panel():
            self._ctrl_expanded = not self._ctrl_expanded
            if self._ctrl_expanded:
                ctrl_expand_frame.grid()
                ctrl_toggle_btn.config(text="折叠控制面板 ▲")
            else:
                ctrl_expand_frame.grid_remove()
                ctrl_toggle_btn.config(text="展开控制面板 ▼")

        ctrl_toggle_btn = ttk.Button(control_frame, text="展开控制面板 ▼",
                                     command=_toggle_ctrl_panel)
        ctrl_toggle_btn.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(4, 2))

        # 可折叠容器（父级仍为 control_frame）
        ctrl_expand_frame = ttk.Frame(control_frame)
        ctrl_expand_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E))
        ctrl_expand_frame.grid_remove()  # 默认折叠
        ctrl_expand_frame.columnconfigure(0, weight=0)
        ctrl_expand_frame.columnconfigure(1, weight=1)
        ctrl_expand_frame.columnconfigure(2, weight=0)

        # 以下所有内容的父级改为 ctrl_expand_frame，行号从 0 重新计数

        # 分隔线
        ttk.Separator(ctrl_expand_frame, orient=tk.HORIZONTAL).grid(
            row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=6)

        # 命令输入
        ttk.Label(ctrl_expand_frame, text="控制命令:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(ctrl_expand_frame, textvariable=self.command_var, width=20)
        self.command_entry.grid(row=1, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        self.command_entry.bind('<Return>', lambda e: self.send_command())

        self.send_btn = ttk.Button(ctrl_expand_frame, text="发送", command=self.send_command, state=tk.DISABLED)
        self.send_btn.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        # 常用命令按钮
        ttk.Label(ctrl_expand_frame, text="常用命令:").grid(row=3, column=0, sticky=tk.W, pady=(10, 5))

        cmd_frame = ttk.Frame(ctrl_expand_frame)
        cmd_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        # 第一行：基础控制
        ttk.Button(cmd_frame, text="使能", command=lambda: self.send_command_text("e"), width=8).grid(row=0, column=0, padx=2)
        ttk.Button(cmd_frame, text="掉电", command=lambda: self.send_command_text("d"), width=8).grid(row=0, column=1, padx=2)
        ttk.Button(cmd_frame, text="读取", command=lambda: self.send_command_text("r"), width=8).grid(row=0, column=2, padx=2)
        ttk.Button(cmd_frame, text="状态", command=lambda: self.send_command_text("s"), width=8).grid(row=0, column=3, padx=2)

        # 第二行：新增按钮
        ttk.Button(cmd_frame, text="站立初始化", command=self.send_stand_init, width=10).grid(row=1, column=1, padx=2, pady=2)
        self.control_loop_btn = ttk.Button(cmd_frame, text="启用控制循环", command=self.toggle_control_loop, width=12)
        self.control_loop_btn.grid(row=1, column=2, padx=2, pady=2)

        # 第三行：电机重置
        ttk.Button(cmd_frame, text="电机重置", command=self.reset_motors, width=10).grid(row=2, column=0, padx=2, pady=2)

        # 分隔线
        ttk.Separator(cmd_frame, orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=4, sticky=(tk.W, tk.E), pady=5)

        # 后处理模块控制
        ttk.Label(cmd_frame, text="后处理模块:", font=('', 9, 'bold')).grid(row=4, column=0, columnspan=4, sticky=tk.W, pady=(5, 2))

        # 髋关节数据模块控制
        self.hip_module_btn = ttk.Button(cmd_frame, text="髋关节数据: 关闭", command=self.toggle_hip_module, width=12)
        self.hip_module_btn.grid(row=5, column=0, columnspan=4, padx=2, pady=2, sticky=(tk.W, tk.E))

        # A1 参数调节（可折叠）
        self.a1_panel_expanded = False
        self.a1_toggle_btn = ttk.Button(
            ctrl_expand_frame, text="A1参数调节 ▶", command=self.toggle_a1_param_panel
        )
        self.a1_toggle_btn.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(6, 2))

        self.a1_param_frame = ttk.LabelFrame(ctrl_expand_frame, text="A1实时参数", padding="6")
        self.a1_param_frame.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(2, 6))
        self.a1_param_frame.grid_remove()  # 默认折叠

        self.a1_param_vars = {
            "ankle_df_th": tk.StringVar(value="18"),
            "hip_ext_th": tk.StringVar(value="-6"),
            "pushoff_max_ms": tk.StringVar(value="300"),
            "ankle_pf_target_deg": tk.StringVar(value="10"),
            "iq_pf_max": tk.StringVar(value="20"),
            "iq_pf_floor": tk.StringVar(value="300"),
            "diq_up_pf": tk.StringVar(value="15"),
        }

        def add_param_row(parent, row, key, label_text):
            ttk.Label(parent, text=label_text, width=16).grid(row=row, column=0, sticky=tk.W, pady=2)
            entry = ttk.Entry(parent, textvariable=self.a1_param_vars[key], width=10)
            entry.grid(row=row, column=1, sticky=tk.W, padx=2, pady=2)
            ttk.Button(
                parent, text="设置", width=6,
                command=lambda k=key: self.set_a1_param(k),
            ).grid(row=row, column=2, padx=2, pady=2)
            ttk.Button(
                parent, text="读取", width=6,
                command=lambda k=key: self.get_a1_param(k),
            ).grid(row=row, column=3, padx=2, pady=2)

        add_param_row(self.a1_param_frame, 0, "ankle_df_th", "ankle_df_th")
        add_param_row(self.a1_param_frame, 1, "hip_ext_th", "hip_ext_th")
        add_param_row(self.a1_param_frame, 2, "pushoff_max_ms", "pushoff_max_ms")
        add_param_row(self.a1_param_frame, 3, "ankle_pf_target_deg", "ankle_pf_target_deg")
        add_param_row(self.a1_param_frame, 4, "iq_pf_max", "iq_pf_max")
        add_param_row(self.a1_param_frame, 5, "iq_pf_floor", "iq_pf_floor")
        add_param_row(self.a1_param_frame, 6, "diq_up_pf", "diq_up_pf")
        ttk.Button(self.a1_param_frame, text="读取全部(params)", command=self.get_a1_params).grid(
            row=7, column=0, columnspan=4, sticky=(tk.W, tk.E), pady=(6, 2)
        )

        # 数据管理
        ttk.Separator(ctrl_expand_frame, orient=tk.HORIZONTAL).grid(
            row=7, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=6)

        data_btn_frame = ttk.Frame(ctrl_expand_frame)
        data_btn_frame.grid(row=8, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        ttk.Button(data_btn_frame, text="载入步态周期", command=self.load_gait_cycle).grid(row=0, column=0, sticky=(tk.W, tk.E), padx=2)
        ttk.Button(data_btn_frame, text="另存为", command=self.save_gait_cycle_as).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=2)
        ttk.Button(data_btn_frame, text="发送到下位机", command=self.send_gait_to_slave).grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=2, pady=2)

        data_btn_frame.columnconfigure(0, weight=1)
        data_btn_frame.columnconfigure(1, weight=1)

        # 指令收发历史（始终可见，行号接在 row=3 的可折叠容器之后）
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=6)

        ttk.Label(control_frame, text="指令收发历史:").grid(
            row=5, column=0, columnspan=3, sticky=tk.W, pady=(5, 2))

        # 创建历史记录文本框（带滚动条）
        history_frame = ttk.Frame(control_frame)
        history_frame.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        control_frame.rowconfigure(6, weight=1)
        
        self.history_text = tk.Text(history_frame, height=10, width=30, wrap=tk.WORD, font=('Consolas', 9))
        scrollbar = ttk.Scrollbar(history_frame, orient=tk.VERTICAL, command=self.history_text.yview)
        self.history_text.configure(yscrollcommand=scrollbar.set)
        
        self.history_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 清空历史按钮
        ttk.Button(control_frame, text="清空历史", command=self.clear_history).grid(row=7, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # 右侧图表区域（跨两行，与患者面板+控制面板对齐）
        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
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
        
        # 创建matplotlib图表（只保留实时数据图，删除步态周期图）
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        # 右侧 Y 轴：只创建一次，后续重复使用，避免在每次刷新时不断叠加新的坐标轴
        self.ax1_right = self.ax1.twinx()
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        
        # 创建标志位复选框区域（替代原来的步态周期图）
        flags_frame = ttk.LabelFrame(plot_frame, text="标志位", padding="10")
        flags_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=False, padx=5, pady=5)
        
        # 数据标志位（第一行）
        data_flags_frame = ttk.Frame(flags_frame)
        data_flags_frame.pack(side=tk.TOP, fill=tk.X, pady=5)
        ttk.Label(data_flags_frame, text="数据标志位:", font=('', 9, 'bold')).pack(side=tk.LEFT, padx=5)
        
        # 定义数据标志位及其说明
        data_flags = [
            ("ph", "步态相位显示值（0/10/20/30），用于离散相位曲线观察"),
            ("s", "步态进度（swing_pct），表示当前步态周期的进度，通常是一个百分比值"),
            ("st", "站立时间或相关时间，可能表示站立阶段的时间"),
            ("ank", "踝关节角度（ankle angle），表示踝关节当前的角度"),
            ("v", "踝关节速度（velocity），表示踝关节的角速度（deg/s）"),
            ("iqT_a", "踝关节目标力矩（target torque），本机型约定：负值=背屈(DF)，正值=跖屈(PF)"),
            ("iqC_a", "踝关节实际力矩（current torque），本机型约定：负值=背屈(DF)，正值=跖屈(PF)"),
            ("hip", "髋关节角度（hip angle），表示髋关节当前的角度"),
            ("hipv", "髋关节速度（hip velocity），表示髋关节的角速度（deg/s）"),
            ("iqT_h", "髋关节目标力矩（target torque），表示期望的髋关节助力大小"),
            ("iqC_h", "髋关节实际力矩（current torque），表示实际输出的髋关节助力值"),
            ("ph4", "四相原始值（0=LOADING,1=MID_STANCE,2=PUSH_OFF,3=SWING）"),
            ("ph4v", "四相放大显示值（ph4*10: 0/10/20/30）"),
            ("ph4p", "四相相内进度（0~1）"),
            ("ph4o", "四相相位曲线输出（辅助比例，0~1）"),
            ("ph4d", "四相退化标志（1=退化模式）"),
        ]
        
        # 存储复选框变量
        self.flag_vars = {}
        
        default_data_flags = {"ph", "s", "st", "ank", "v"}

        # 创建数据标志位复选框
        for i, (flag_name, tooltip_text) in enumerate(data_flags):
            var = tk.BooleanVar(value=(flag_name in default_data_flags))
            self.flag_vars[flag_name] = var
            
            # 创建复选框框架
            check_frame = ttk.Frame(data_flags_frame)
            check_frame.pack(side=tk.LEFT, padx=3)
            
            # 复选框
            cb = ttk.Checkbutton(check_frame, text=flag_name, variable=var)
            cb.pack(side=tk.LEFT)
            
            # 问号标签（带提示）
            help_label = ttk.Label(check_frame, text="?", cursor="hand2", 
                                   foreground="blue", font=('', 9, 'bold'))
            help_label.pack(side=tk.LEFT, padx=(2, 0))
            
            # 绑定提示（使用ToolTip或简单messagebox）
            def create_tooltip(widget, tooltip_text_local):
                def show_tooltip(event):
                    tooltip = tk.Toplevel()
                    tooltip.wm_overrideredirect(True)
                    tooltip.wm_geometry(f"+{event.x_root+10}+{event.y_root+10}")
                    label = ttk.Label(tooltip, text=tooltip_text_local, background="#ffffe0", 
                                     relief=tk.SOLID, borderwidth=1, 
                                     font=('', 9), wraplength=300)
                    label.pack()
                    tooltip.after(5000, tooltip.destroy)  # 5秒后自动关闭
                widget.bind("<Button-1>", show_tooltip)
            
            create_tooltip(help_label, tooltip_text)
        
        # 状态标志位（第二行）
        state_flags_frame = ttk.Frame(flags_frame)
        state_flags_frame.pack(side=tk.TOP, fill=tk.X, pady=5)
        ttk.Label(state_flags_frame, text="状态标志位:", font=('', 9, 'bold')).pack(side=tk.LEFT, padx=5)
        
        # 定义状态标志位及其说明
        state_flags = [
            ("PF", "跖屈助力（push-off）状态，表示是否处于跖屈助力阶段"),
            ("DF", "背屈助力（swing DF）状态，表示是否处于背屈助力阶段"),
            ("UL", "卸载（unload）状态，表示是否处于卸载阶段"),
            ("comp", "退让（compliant）模式状态，表示是否处于退让模式"),
            ("cool", "冷却模式状态，表示是否处于冷却模式"),
            ("abn", "异常状态，表示是否处于异常状态"),
        ]
        
        # 创建状态标志位复选框
        for i, (flag_name, tooltip_text) in enumerate(state_flags):
            var = tk.BooleanVar(value=False)
            self.flag_vars[flag_name] = var
            
            # 创建复选框框架
            check_frame = ttk.Frame(state_flags_frame)
            check_frame.pack(side=tk.LEFT, padx=3)
            
            # 复选框
            cb = ttk.Checkbutton(check_frame, text=flag_name, variable=var)
            cb.pack(side=tk.LEFT)
            
            # 问号标签（带提示）
            help_label = ttk.Label(check_frame, text="?", cursor="hand2", 
                                   foreground="blue", font=('', 9, 'bold'))
            help_label.pack(side=tk.LEFT, padx=(2, 0))
            
            # 绑定提示
            def create_tooltip(widget, tooltip_text_local):
                def show_tooltip(event):
                    tooltip = tk.Toplevel()
                    tooltip.wm_overrideredirect(True)
                    tooltip.wm_geometry(f"+{event.x_root+10}+{event.y_root+10}")
                    label = ttk.Label(tooltip, text=tooltip_text_local, background="#ffffe0", 
                                     relief=tk.SOLID, borderwidth=1, 
                                     font=('', 9), wraplength=300)
                    label.pack()
                    tooltip.after(5000, tooltip.destroy)  # 5秒后自动关闭
                widget.bind("<Button-1>", show_tooltip)
            
            create_tooltip(help_label, tooltip_text)
        
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

    def toggle_a1_param_panel(self):
        """展开/折叠 A1 参数调节区域"""
        self.a1_panel_expanded = not self.a1_panel_expanded
        if self.a1_panel_expanded:
            self.a1_param_frame.grid()
            self.a1_toggle_btn.config(text="A1参数调节 ▼")
        else:
            self.a1_param_frame.grid_remove()
            self.a1_toggle_btn.config(text="A1参数调节 ▶")

    def set_a1_param(self, key):
        """发送 set <name> <value> 命令"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "请先连接串口")
            return
        value = self.a1_param_vars[key].get().strip()
        if not value:
            messagebox.showwarning("提示", f"{key} 不能为空")
            return
        cmd = f"set {key} {value}"
        try:
            self.collector.send_command(cmd)
            self.add_history(cmd, "TX")
        except Exception as e:
            messagebox.showerror("错误", str(e))

    def get_a1_param(self, key):
        """发送 get <name> 命令"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "请先连接串口")
            return
        cmd = f"get {key}"
        try:
            self.collector.send_command(cmd)
            self.add_history(cmd, "TX")
        except Exception as e:
            messagebox.showerror("错误", str(e))

    def get_a1_params(self):
        """发送 params 命令读取所有 A1 参数"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "请先连接串口")
            return
        try:
            self.collector.send_command("params")
            self.add_history("params", "TX")
        except Exception as e:
            messagebox.showerror("错误", str(e))
    
    def reset_motors(self):
        """电机重置：向两个电机发送clearerror指令，然后发送enable指令"""
        if not self.collector.is_connected():
            messagebox.showerror("错误", "串口未连接")
            return
        
        try:
            # 清除两个电机的错误标志（发送CAN命令0x9B）
            self.collector.send_command("ce")
            self.add_history("ce (clear error)", "TX")
            time.sleep(0.1)  # 短暂延迟
            
            # 使能两个电机（发送CAN命令0x88）
            self.collector.send_command("e")
            self.add_history("e (enable)", "TX")
            
            messagebox.showinfo("成功", "电机重置完成")
        except Exception as e:
            error_msg = f"错误: {str(e)}"
            self.add_history(error_msg, "信息")
            messagebox.showerror("错误", str(e))
    
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
        # 重置实时数据图表
        self.ax1.relim()
        self.ax1.autoscale()
        if hasattr(self, "ax1_right") and self.ax1_right is not None:
            self.ax1_right.relim()
            self.ax1_right.autoscale()
        
        # 更新图表显示
        self.canvas.draw()
        self.add_history("图表已重置为显示全部", "信息")
    
    def clear_data(self):
        """清除所有采集的数据和曲线"""
        # 确认操作
        if messagebox.askyesno("确认", "确定要清除所有采集的数据和曲线吗？"):
            # 清除数据
            self.collector.clear_all_data()
            # 清除后重置绘制状态，避免残留增量更新状态造成“延迟感”
            self._plot_initialized = False
            self._last_realtime_len = 0
            self._last_received_count = self.collector.total_received
            self._plot_lines = {'hip_f': None}
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

        # ax1.clear() 会清掉十字准星线对象，需要重建
        self._reset_crosshair()

        time_data, hip_filtered = self.collector.get_realtime_data()
        if len(time_data) > 0 and len(hip_filtered) > 0 and len(time_data) == len(hip_filtered):
            # 仅绘制滤波后的髋角（hip_f）
            hip_filtered_valid = [(x if x is not None else float('nan')) for x in hip_filtered]
            if any(not np.isnan(x) for x in hip_filtered_valid):
                self.ax1.plot(time_data, hip_filtered_valid, 'r--', label='髋关节滤波(hip_f)', linewidth=1.2, alpha=0.8)
            
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
            
            self.ax1.set_title('实时数据（根据复选框选择显示）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度/速度/力矩', color='black')
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
            self.ax1.set_title('实时数据（根据复选框选择显示）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度/速度/力矩')
            self.ax1.grid(True)
        
        # 已删除步态周期图，不再更新
        
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
        # 只显示实时数据图
        self.ax1.set_title('实时数据（髋关节和踝关节角度，髋关节速度）', fontsize=12)
        self.ax1.set_xlabel('时间 (秒)')
        self.ax1.set_ylabel('角度 (度)')
        self.ax1.grid(True)
        
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
        self.canvas.mpl_connect('axes_leave_event', self.on_axes_leave)
        self.canvas.mpl_connect('figure_leave_event', self.on_axes_leave)

        # 初始化十字准星线（默认隐藏）
        self._init_crosshair()

    def _init_crosshair(self):
        """创建十字准星的两条虚线（竖线x/横线y）。"""
        try:
            # 避免重复创建（比如重建figure/axes）
            if self._crosshair_vline is not None and self._crosshair_hline is not None:
                return
            # 用 ax1 作为绘制载体（横线/竖线覆盖整个绘图区）
            self._crosshair_vline = self.ax1.axvline(
                x=0.0, color='#888888', linestyle='--', linewidth=1.0, alpha=0.9, visible=False, zorder=50
            )
            self._crosshair_hline = self.ax1.axhline(
                y=0.0, color='#888888', linestyle='--', linewidth=1.0, alpha=0.9, visible=False, zorder=50
            )
        except Exception:
            # 交互不是核心功能，避免初始化失败影响主流程
            self._crosshair_vline = None
            self._crosshair_hline = None

    def _reset_crosshair(self):
        """在 ax.clear() 之后重建十字准星线对象。"""
        self._crosshair_vline = None
        self._crosshair_hline = None
        self._init_crosshair()

    def _update_crosshair(self, event):
        """根据鼠标位置更新十字准星。"""
        if self._crosshair_vline is None or self._crosshair_hline is None:
            return
        if event.inaxes is None:
            self._crosshair_vline.set_visible(False)
            self._crosshair_hline.set_visible(False)
            return

        # 只在实时曲线区域显示（ax1 或其右轴 ax1_right）
        if event.inaxes != self.ax1 and not (hasattr(self, "ax1_right") and event.inaxes == self.ax1_right):
            self._crosshair_vline.set_visible(False)
            self._crosshair_hline.set_visible(False)
            return

        # x 直接用 event.xdata；y 统一转换成 ax1 的数据坐标，保证横线对齐左轴刻度
        x = event.xdata
        if x is None:
            return
        # event.x/event.y 是像素坐标，可用于跨轴转换
        try:
            _, y_ax1 = self.ax1.transData.inverted().transform((event.x, event.y))
        except Exception:
            y_ax1 = event.ydata

        if y_ax1 is None:
            return

        self._crosshair_vline.set_xdata([x, x])
        self._crosshair_hline.set_ydata([y_ax1, y_ax1])
        self._crosshair_vline.set_visible(True)
        self._crosshair_hline.set_visible(True)

    def on_axes_leave(self, event):
        """鼠标离开坐标轴/画布时隐藏十字准星。"""
        if self._crosshair_vline is not None:
            self._crosshair_vline.set_visible(False)
        if self._crosshair_hline is not None:
            self._crosshair_hline.set_visible(False)
        self.canvas.draw_idle()
    
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
    
    def on_release(self, event):
        """鼠标释放事件"""
        self.pan_active = False
        self.zoom_active = False
        self.press_x = None
        self.press_y = None
    
    def on_motion(self, event):
        """鼠标移动事件（用于十字准星与平移）"""
        # 十字准星：无论是否平移，都尝试更新
        self._update_crosshair(event)

        # 平移：仅在右键/中键按住时生效
        if not self.pan_active or event.inaxes is None:
            # 只更新十字准星时，不需要每次都重绘大量内容
            self.canvas.draw_idle()
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
        
        if ax1_bbox:
            # 检查点是否在ax1的bbox内
            ax1_x0, ax1_y0 = ax1_bbox.x0, ax1_bbox.y0
            ax1_x1, ax1_y1 = ax1_bbox.x1, ax1_bbox.y1
            
            if ax1_x0 <= fig_x <= ax1_x1 and ax1_y0 <= fig_y <= ax1_y1:
                ax = self.ax1
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
    
    def _feed_training_session(self):
        """
        将 collector 接收到的最新数据同步给 TrainingSession，
        同时更新训练统计显示，并每 5 秒刷新一次临时文件。
        仅在训练会话运行或暂停时执行。
        """
        session = self.training_session
        if session.state not in (TrainingSession.STATE_RUNNING,
                                  TrainingSession.STATE_PAUSED):
            return

        # 从 _hip_process_loop 存储的数据缓冲中取最新帧（非破坏性读取）
        # 注：collector.data_queue 已被 _hip_process_loop 消费，这里改用 collector 的
        # 各数据 deque 推算最新帧。因为我们需要完整的每一帧数据供 TrainingSession 记录，
        # 所以在 _hip_process_loop 处理前拦截是最优的。
        # 实际上 collector 的 data_queue 由 _hip_process_loop 消费，
        # 我们通过记录 collector 已处理数量来批量抓取新帧。
        if not hasattr(self, '_session_last_data_len'):
            self._session_last_data_len = 0

        current_len = len(self.collector.time_data)
        if session.state == TrainingSession.STATE_RUNNING and current_len > self._session_last_data_len:
            new_start = self._session_last_data_len
            new_count = current_len - new_start

            # 从各 deque 截取新帧（deque 为环形缓冲，使用 list 切片）
            def _slice(deque_obj, start, count):
                lst = list(deque_obj)
                return lst[max(0, len(lst) - new_count):] if start == 0 else lst[-count:]

            ank_list = list(self.collector.ank_data)[-new_count:] if new_count > 0 else []
            hip_list = list(self.collector.hip_data_new)[-new_count:] if new_count > 0 else []
            ph_list = list(self.collector.ph_data)[-new_count:] if new_count > 0 else []
            t_list = list(self.collector.time_data)[-new_count:] if new_count > 0 else []

            for i in range(new_count):
                frame = {
                    "t": t_list[i] if i < len(t_list) else None,
                    "ank": ank_list[i] if i < len(ank_list) else None,
                    "hip": hip_list[i] if i < len(hip_list) else None,
                    "ph": ph_list[i] if i < len(ph_list) else None,
                }
                session.add_data(frame)

        self._session_last_data_len = current_len

        # 刷新统计标签
        if hasattr(self, '_duration_var'):
            self._update_training_stats_ui()

        # 每 5 秒刷新一次临时文件
        now = time.time()
        if now - self._last_flush_check >= 5.0:
            self._last_flush_check = now
            session.flush_temp()

    def _process_torque_data(self):
        """处理转矩控制阶段的新格式数据（已由_hip_process_loop存储，这里不需要额外处理）"""
        # 新格式数据已经在_hip_process_loop中无条件存储到专用队列
        # 绘图时根据复选框状态决定是否显示
        pass
    
    def update_plots(self):
        """更新图表（使用增量更新而非完全重绘，显著提升性能）"""
        # ── 训练会话：将 data_queue 中的数据同步给 TrainingSession ──
        self._feed_training_session()

        # 处理新格式数据（根据复选框状态）
        self._process_torque_data()
        
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
        
        # 获取实时数据（仅保留hip_f；其余曲线由复选框控制的新格式数据提供）
        time_data, hip_filtered = self.collector.get_realtime_data()
        
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
            self._plot_lines = {'hip_f': None}

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
                    self._reset_crosshair()
                    
                    # 确保数据长度一致
                    min_len = min(len(time_data), len(hip_filtered) if hip_filtered else 0)
                    if min_len > 0:
                        # 截取到最小长度
                        time_arr = np.array(list(time_data)[-min_len:])
                        
                        # 绘制滤波后的髋角（如果有效）
                        if hip_filtered and len(hip_filtered) > 0:
                            valid_indices = [i for i, x in enumerate(hip_filtered[-min_len:]) if x is not None]
                            if len(valid_indices) > 0:
                                valid_time = time_arr[valid_indices]
                                valid_filtered = np.array([hip_filtered[i] for i in valid_indices])
                                line_f, = self.ax1.plot(valid_time, valid_filtered, 'r--', 
                                                       label='髋关节滤波(hip_f)', linewidth=1.5, alpha=0.8)
                                self._plot_lines['hip_f'] = line_f
                        
                        # 根据复选框状态绘制新格式数据
                        if hasattr(self, 'flag_vars'):
                            # 定义数据标志位及其颜色和标签
                            data_flag_config = {
                                'ph': ('ph_data', '#FF0000', '步态相位(ph)', '-'),
                                's': ('s_data', '#00FF00', '步态进度(s)', '-'),
                                'st': ('st_data', '#0000FF', '站立时间(st)', '-'),
                                'ank': ('ank_data', '#FF00FF', '踝关节角度(ank)', '-'),
                                'v': ('v_data', '#00FFFF', '踝关节速度(v)', '--'),
                                'iqT_a': ('iqT_a_data', '#FFFF00', '踝目标力矩(iqT_a, -DF/+PF)', '-'),
                                'iqC_a': ('iqC_a_data', '#FF8800', '踝实际力矩(iqC_a, -DF/+PF)', '--'),
                                'hip': ('hip_data_new', '#0088FF', '髋关节角度(hip)', '-'),
                                'hipv': ('hipv_data', '#88FF00', '髋关节速度(hipv)', '--'),
                                'iqT_h': ('iqT_h_data', '#FF0088', '髋目标力矩(iqT_h)', '-'),
                                'iqC_h': ('iqC_h_data', '#88FF88', '髋实际力矩(iqC_h)', '--'),
                                'ph4': ('ph4_data', '#AAAAAA', '四相原始值(ph4)', '-'),
                                'ph4v': ('ph4v_data', '#FF4444', '四相放大值(ph4v)', '-'),
                                'ph4p': ('ph4p_data', '#44AAFF', '四相进度(ph4p)', '--'),
                                'ph4o': ('ph4o_data', '#AA44FF', '四相输出(ph4o)', '-'),
                                'ph4d': ('ph4d_data', '#222222', '四相退化(ph4d)', '-'),
                            }
                            
                            # 获取时间数据（用于新格式数据）
                            if len(self.collector.time_data) > 0:
                                latest_time = self.collector.time_data[-1]
                                time_arr_new = np.array([(t - latest_time) / 1000.0 for t in self.collector.time_data])
                                
                                # 绘制选中的数据标志位
                                for flag_name, (deque_name, color, label, linestyle) in data_flag_config.items():
                                    if flag_name in self.flag_vars and self.flag_vars[flag_name].get():
                                        if hasattr(self.collector, deque_name):
                                            data_deque = getattr(self.collector, deque_name)
                                            if len(data_deque) > 0:
                                                # 确保数据长度与时间数据一致
                                                time_len = len(self.collector.time_data)
                                                data_list = list(data_deque)
                                                
                                                # 确保长度匹配（取较小值，避免索引错误）
                                                actual_len = min(len(data_list), time_len, len(time_arr_new))
                                                if actual_len == 0:
                                                    continue
                                                
                                                # 截取到实际长度
                                                data_list = data_list[:actual_len]
                                                time_arr_plot = time_arr_new[:actual_len]
                                                
                                                # 转换为numpy数组并过滤有效数据
                                                data_arr = np.array([x if x is not None else np.nan for x in data_list])
                                                valid_mask = ~np.isnan(data_arr)
                                                
                                                # 确保valid_mask和time_arr_plot长度一致
                                                if len(valid_mask) == len(time_arr_plot) and np.any(valid_mask):
                                                    line_key = f'torque_{flag_name}'
                                                    line, = self.ax1.plot(time_arr_plot[valid_mask], data_arr[valid_mask], 
                                                                         color=color, linestyle=linestyle, 
                                                                         linewidth=1.5, label=label, alpha=0.8)
                                                    self._plot_lines[line_key] = line
                                                elif len(valid_mask) != len(time_arr_plot):
                                                    # 长度不匹配，跳过绘制（避免索引错误）
                                                    # 只在调试时打印，避免刷屏
                                                    pass
                        
                        self.ax1.set_title('实时数据（根据复选框选择显示）', fontsize=12)
                        self.ax1.set_xlabel('时间 (秒)')
                        self.ax1.set_ylabel('角度/速度/力矩', color='black')
                        self.ax1.grid(True, alpha=0.3)
                        self.ax1.legend(loc='upper right', fontsize=8)
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
                else:
                    # 增量更新：仅更新数据而不清空重绘
                    if self._plot_lines:
                        min_len = min(len(time_data), len(hip_filtered) if hip_filtered else 0)
                        if min_len <= 0:
                            return
                        time_arr = np.array(list(time_data)[-min_len:])
                        
                        # 更新滤波后的髋角
                        if hip_filtered and len(hip_filtered) > 0 and 'hip_f' in self._plot_lines:
                            valid_indices = [i for i, x in enumerate(hip_filtered[-min_len:]) if x is not None]
                            if len(valid_indices) > 0:
                                valid_time = time_arr[valid_indices]
                                valid_filtered = np.array([hip_filtered[i] for i in valid_indices])
                                self._plot_lines['hip_f'].set_data(valid_time, valid_filtered)
                        
                        # 增量更新新格式数据（根据复选框状态）
                        if hasattr(self, 'flag_vars') and len(self.collector.time_data) > 0:
                            latest_time = self.collector.time_data[-1]
                            time_arr_new = np.array([(t - latest_time) / 1000.0 for t in self.collector.time_data])
                            
                            data_flag_config = {
                                'ph': ('ph_data', '#FF0000', '步态相位(ph)', '-'),
                                's': ('s_data', '#00FF00', '步态进度(s)', '-'),
                                'st': ('st_data', '#0000FF', '站立时间(st)', '-'),
                                'ank': ('ank_data', '#FF00FF', '踝关节角度(ank)', '-'),
                                'v': ('v_data', '#00FFFF', '踝关节速度(v)', '--'),
                                'iqT_a': ('iqT_a_data', '#FFFF00', '踝目标力矩(iqT_a, -DF/+PF)', '-'),
                                'iqC_a': ('iqC_a_data', '#FF8800', '踝实际力矩(iqC_a, -DF/+PF)', '--'),
                                'hip': ('hip_data_new', '#0088FF', '髋关节角度(hip)', '-'),
                                'hipv': ('hipv_data', '#88FF00', '髋关节速度(hipv)', '--'),
                                'iqT_h': ('iqT_h_data', '#FF0088', '髋目标力矩(iqT_h)', '-'),
                                'iqC_h': ('iqC_h_data', '#88FF88', '髋实际力矩(iqC_h)', '--'),
                                'ph4': ('ph4_data', '#AAAAAA', '四相原始值(ph4)', '-'),
                                'ph4v': ('ph4v_data', '#FF4444', '四相放大值(ph4v)', '-'),
                                'ph4p': ('ph4p_data', '#44AAFF', '四相进度(ph4p)', '--'),
                                'ph4o': ('ph4o_data', '#AA44FF', '四相输出(ph4o)', '-'),
                                'ph4d': ('ph4d_data', '#222222', '四相退化(ph4d)', '-'),
                            }
                            
                            for flag_name, (deque_name, color, label, linestyle) in data_flag_config.items():
                                if flag_name in self.flag_vars and self.flag_vars[flag_name].get():
                                    line_key = f'torque_{flag_name}'
                                    if line_key in self._plot_lines and hasattr(self.collector, deque_name):
                                        data_deque = getattr(self.collector, deque_name)
                                        if len(data_deque) > 0:
                                            time_len = len(self.collector.time_data)
                                            data_list = list(data_deque)
                                            actual_len = min(len(data_list), time_len, len(time_arr_new))
                                            if actual_len > 0:
                                                data_list = data_list[:actual_len]
                                                time_arr_plot = time_arr_new[:actual_len]
                                                data_arr = np.array([x if x is not None else np.nan for x in data_list])
                                                valid_mask = ~np.isnan(data_arr)
                                                if len(valid_mask) == len(time_arr_plot) and np.any(valid_mask):
                                                    self._plot_lines[line_key].set_data(time_arr_plot[valid_mask], data_arr[valid_mask])
                        
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
                    self._reset_crosshair()
                    # ✓ 增强调试信息
                    hip_status = "✓" if self.collector.hip_module_enabled else "✗"
                    debug_info = (
                        f'等待数据...\n'
                        f'队列大小: {queue_size} | 收到: {self.collector.total_received}\n'
                        f'数据点数: {total_points}\n'
                        f'髋关节模块: {hip_status}\n'
                        f'[提示] 点击"髋关节数据"启动数据处理'
                    )
                    self.ax1.text(0.5, 0.5, debug_info, 
                                 horizontalalignment='center', verticalalignment='center',
                                 transform=self.ax1.transAxes, fontsize=11)
                    self.ax1.set_title('实时数据（根据复选框选择显示）', fontsize=12)
                    self.ax1.set_xlabel('时间 (秒)')
                    self.ax1.set_ylabel('角度/速度/力矩')
                    self.ax1.grid(True)
        
        # 已删除步态周期图，不再更新
        
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
            (self._last_realtime_len == 0 and new_len > 0)  # 从空白状态恢复
        )
        
        # ✓ 诊断：绘制决策已在内部处理
        if not hasattr(self, '_last_draw_log_time'):
            self._last_draw_log_time = time.time()
        current_time = time.time()
        
        # 更新诊断时间戳
        if current_time - self._last_draw_log_time > 2.0:
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
    
    def toggle_hip_module(self):
        """切换髋关节数据模块"""
        if self.collector.hip_module_enabled:
            # 停止髋关节数据模块
            self.collector.stop_hip_module()
            # 关闭髋关节数据模块时，自动停止下位机 gait JSON 上传
            self.collector.send_command("gcs")
            self.add_history("gcs", "TX")
            self.hip_module_btn.config(text="髋关节数据: 关闭")
            self.add_history("髋关节数据模块已关闭（已自动发送 gcs）", "信息")
        else:
            # 启动髋关节数据模块
            if not self.collector.is_connected():
                messagebox.showerror("错误", "请先连接串口")
                return
            
            try:
                self.collector.start_hip_module()
                # 开启髋关节数据模块时，自动触发下位机开始发送 gait JSON 数据
                self.collector.send_command("gc")
                self.add_history("gc", "TX")
                # ✓ 重置GUI绘制状态，准备新的数据显示
                self._plot_initialized = False
                self._last_realtime_len = 0
                self._plot_lines = {'hip_f': None}
                self.hip_module_btn.config(text="髋关节数据: 开启")
                self.add_history("髋关节数据模块已开启（已自动发送 gc）", "信息")
            except Exception as e:
                error_msg = f"启动髋关节数据模块失败: {str(e)}"
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
