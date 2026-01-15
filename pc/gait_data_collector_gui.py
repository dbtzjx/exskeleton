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
        self.data_queue = queue.Queue()
        self.raw_data_queue = queue.Queue()  # 原始数据队列（用于历史记录）
        self.is_collecting = False
        self.collect_thread = None
        
        # 数据存储
        self.time_data = deque(maxlen=MAX_DATA_POINTS)
        self.hip_data = deque(maxlen=MAX_DATA_POINTS)
        self.ankle_data = deque(maxlen=MAX_DATA_POINTS)
        
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
    
    def start_collection(self):
        """启动数据采集"""
        if not self.is_connected():
            raise Exception("串口未连接")
        
        if self.is_collecting:
            return False
        
        self.is_collecting = True
        self.collect_thread = threading.Thread(target=self._collect_data, daemon=True)
        self.collect_thread.start()
        return True
    
    def stop_collection(self):
        """停止数据采集"""
        self.is_collecting = False
        if self.collect_thread:
            self.collect_thread.join(timeout=2)
    
    def _collect_data(self):
        """数据采集线程（从串口读取数据）"""
        buffer = ""
        while self.is_collecting:
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
                                    if 't' in data_dict and 'h' in data_dict and 'a' in data_dict:
                                        self.data_queue.put(data_dict)
                                        self.total_received += 1
                                        self.last_received_time = time.time()
                                    else:
                                        # JSON格式正确但缺少必要字段，打印调试信息（仅在前几次）
                                        if self.is_collecting and self.total_received < 5:
                                            print(f"警告: JSON缺少必要字段: {json_str}")
                                except json.JSONDecodeError as e:
                                    # JSON解析失败，打印调试信息（仅在前几次）
                                    if self.is_collecting and self.total_received < 5:
                                        print(f"JSON解析失败: {json_str[:50]}..., 错误: {e}")
                
                time.sleep(0.01)  # 避免CPU占用过高
            except Exception as e:
                if self.is_collecting:
                    print(f"数据采集错误: {e}")
                break
    
    def get_raw_data(self):
        """获取原始返回数据（用于历史记录）"""
        raw_lines = []
        while not self.raw_data_queue.empty():
            try:
                raw_lines.append(self.raw_data_queue.get_nowait())
            except queue.Empty:
                break
        return raw_lines
    
    def process_data(self):
        """处理接收到的数据（在主线程中调用）"""
        count = 0
        new_data_count = 0
        while not self.data_queue.empty() and count < 100:  # 每次最多处理100条
            try:
                data = self.data_queue.get_nowait()
                timestamp = data['t']  # 毫秒
                hip_angle = data['h']   # 度
                ankle_angle = data['a'] # 度
                
                # 添加到实时数据
                self.time_data.append(timestamp)
                self.hip_data.append(hip_angle)
                self.ankle_data.append(ankle_angle)
                
                # 步态周期识别（基于髋关节角度）
                self._detect_gait_cycle(timestamp, hip_angle, ankle_angle)
                count += 1
                new_data_count += 1
            except queue.Empty:
                break
            except Exception as e:
                # 数据解析错误，跳过这条数据
                print(f"处理数据错误: {e}")
                break
    
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
                
                self.time_data.clear()
                self.hip_data.clear()
                self.ankle_data.clear()
                
                for i, rel_time in enumerate(time_list):
                    timestamp = base_time + int(rel_time * 1000) - start_offset
                    self.time_data.append(timestamp)
                    self.hip_data.append(data.get('hip_angle', [])[i])
                    self.ankle_data.append(data.get('ankle_angle', [])[i])
            
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
        """获取实时数据（用于绘图）"""
        if len(self.time_data) == 0 or len(self.hip_data) == 0 or len(self.ankle_data) == 0:
            return [], [], []
        
        # 确保数据长度一致
        min_len = min(len(self.time_data), len(self.hip_data), len(self.ankle_data))
        if min_len == 0:
            return [], [], []
        
        # 转换为相对时间（从最新数据往前）
        latest_time = self.time_data[-1]
        relative_time = [(self.time_data[i] - latest_time) / 1000.0 for i in range(min_len)]  # 转换为秒
        hip_data = [self.hip_data[i] for i in range(min_len)]
        ankle_data = [self.ankle_data[i] for i in range(min_len)]
        
        return relative_time, hip_data, ankle_data
    
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
        self.root = root
        self.root.title("步态数据采集与可视化")
        self.root.geometry("1200x800")
        
        self.collector = GaitDataCollector()
        self.update_interval = 50  # 更新间隔（毫秒）
        self.history_max_lines = 500  # 历史记录最大行数
        
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
        
        ttk.Button(cmd_frame, text="使能", command=lambda: self.send_command_text("e"), width=8).grid(row=0, column=0, padx=2)
        ttk.Button(cmd_frame, text="掉电", command=lambda: self.send_command_text("d"), width=8).grid(row=0, column=1, padx=2)
        ttk.Button(cmd_frame, text="读取", command=lambda: self.send_command_text("r"), width=8).grid(row=0, column=2, padx=2)
        ttk.Button(cmd_frame, text="状态", command=lambda: self.send_command_text("s"), width=8).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(cmd_frame, text="开始采集", command=lambda: self.send_command_text("gc"), width=8).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(cmd_frame, text="停止采集", command=lambda: self.send_command_text("gcs"), width=8).grid(row=1, column=2, padx=2, pady=2)
        
        # 数据采集控制
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=7, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        # 数据采集状态显示
        self.collect_status_label = ttk.Label(control_frame, text="采集状态: 未启动", foreground="gray")
        self.collect_status_label.grid(row=8, column=0, columnspan=3, sticky=tk.W, pady=2)
        
        self.collect_btn = ttk.Button(control_frame, text="开始数据采集", command=self.toggle_collection, state=tk.DISABLED)
        self.collect_btn.grid(row=9, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
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
        
        # 创建matplotlib图表
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        
        # 工具栏
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        self.toolbar.pack(side=tk.LEFT, fill=tk.X)
        
        # 添加清除数据按钮
        clear_btn = ttk.Button(toolbar_frame, text="清除数据", command=self.clear_data)
        clear_btn.pack(side=tk.RIGHT, padx=5)
        
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
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
            if self.collector.is_collecting:
                self.collector.stop_collection()
                self.collect_btn.config(text="开始数据采集", state=tk.DISABLED)
                self.add_history("数据采集已停止", "信息")
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
                self.collect_btn.config(state=tk.NORMAL)
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
    
    def toggle_collection(self):
        """切换数据采集状态"""
        if self.collector.is_collecting:
            # 停止采集
            # 先停止下位机采集
            if self.collector.is_connected():
                try:
                    self.collector.send_command("gcs")
                    self.add_history("gcs", "TX")
                except Exception as e:
                    self.add_history(f"停止下位机采集失败: {str(e)}", "信息")
            
            # 停止上位机采集
            self.collector.stop_collection()
            self.collect_btn.config(text="开始数据采集")
            self.collect_status_label.config(text="采集状态: 已停止", foreground="gray")
            self.add_history("数据采集已停止", "信息")
        else:
            # 开始采集
            if not self.collector.is_connected():
                messagebox.showerror("错误", "请先连接串口")
                return
            
            try:
                # 先启动上位机采集
                self.collector.start_collection()
                self.collect_btn.config(text="停止数据采集")
                self.collect_status_label.config(text="采集状态: 运行中...", foreground="green")
                self.add_history("数据采集已启动", "信息")
                
                # 自动发送'gc'命令启动下位机采集
                try:
                    self.collector.send_command("gc")
                    self.add_history("gc", "TX")
                    self.add_history("已自动启动下位机数据采集", "信息")
                except Exception as e:
                    error_msg = f"启动下位机采集失败: {str(e)}"
                    self.add_history(error_msg, "信息")
                    # 如果下位机启动失败，停止上位机采集
                    self.collector.stop_collection()
                    self.collect_btn.config(text="开始数据采集")
                    self.collect_status_label.config(text="采集状态: 启动失败", foreground="red")
                    messagebox.showerror("错误", f"启动下位机采集失败: {str(e)}")
            except Exception as e:
                error_msg = f"启动采集失败: {str(e)}"
                self.add_history(error_msg, "信息")
                self.collect_status_label.config(text="采集状态: 启动失败", foreground="red")
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
            cycle_file = os.path.join(folder_path, "gait_cycle.json")
            realtime_file = os.path.join(folder_path, "realtime_data.json")
            
            loaded_files = []
            
            # 载入步态周期数据
            if os.path.exists(cycle_file):
                self.collector.load_gait_cycle(cycle_file)
                loaded_files.append("步态周期数据")
            
            # 载入实时数据
            if os.path.exists(realtime_file):
                self.collector.load_realtime_data(realtime_file)
                loaded_files.append("实时数据")
            
            if not loaded_files:
                messagebox.showwarning("警告", f"文件夹中没有找到数据文件:\n{gait_cycle.json}\n或\nrealtime_data.json")
                return
            
            # 显示载入成功信息
            message_text = "数据已载入:\n" + "\n".join(loaded_files)
            messagebox.showinfo("成功", message_text)
            self.add_history(f"数据已从 {folder_path} 载入", "信息")
            
            # 强制更新图表（清除缓存）
            self._last_cycle_len = -1
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
    
    def clear_data(self):
        """清除所有采集的数据和曲线"""
        # 确认操作
        if messagebox.askyesno("确认", "确定要清除所有采集的数据和曲线吗？"):
            # 清除数据
            self.collector.clear_all_data()
            # 重置周期长度缓存
            self._last_cycle_len = -1
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
        
        self.ax1.clear()
        time_data, hip_data, ankle_data = self.collector.get_realtime_data()
        if len(time_data) > 0 and len(hip_data) > 0 and len(ankle_data) > 0 and len(time_data) == len(hip_data) == len(ankle_data):
            self.ax1.plot(time_data, hip_data, 'b-', label='髋关节', linewidth=1.5)
            self.ax1.plot(time_data, ankle_data, 'r-', label='踝关节', linewidth=1.5)
            self.ax1.set_title('实时数据（髋关节和踝关节角度）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度 (度)')
            self.ax1.grid(True)
            self.ax1.legend(loc='upper right')
            if not auto_scale_x1 and xlim1 is not None:
                self.ax1.set_xlim(xlim1)
            else:
                self.ax1.relim()
                self.ax1.autoscale()
        else:
            self.ax1.text(0.5, 0.5, '等待数据...', 
                         horizontalalignment='center', verticalalignment='center',
                         transform=self.ax1.transAxes, fontsize=14)
            self.ax1.set_title('实时数据（髋关节和踝关节角度）', fontsize=12)
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
        # 检查是否有数据可保存
        has_cycle_data = (len(self.collector.gait_cycle_time) > 0 and 
                         len(self.collector.gait_cycle_hip) > 0 and 
                         len(self.collector.gait_cycle_ankle) > 0)
        
        has_realtime_data = (len(self.collector.time_data) > 0 and 
                            len(self.collector.hip_data) > 0 and 
                            len(self.collector.ankle_data) > 0)
        
        if not has_cycle_data and not has_realtime_data:
            messagebox.showwarning("警告", "没有可保存的数据")
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
            
            # 保存实时数据
            if has_realtime_data:
                realtime_file = os.path.join(folder_path, "realtime_data.json")
                if len(self.collector.time_data) > 0:
                    start_time = self.collector.time_data[0]
                    relative_time = [(t - start_time) / 1000.0 for t in self.collector.time_data]
                else:
                    relative_time = []
                
                realtime_data = {
                    "timestamp": datetime.now().isoformat(),
                    "data_type": "realtime",
                    "duration": (self.collector.time_data[-1] - self.collector.time_data[0]) / 1000.0 if len(self.collector.time_data) > 1 else 0.0,
                    "data_points": len(self.collector.time_data),
                    "time": relative_time,
                    "hip_angle": list(self.collector.hip_data),
                    "ankle_angle": list(self.collector.ankle_data)
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
        self.ax1.set_title('实时数据（髋关节和踝关节角度）', fontsize=12)
        self.ax1.set_xlabel('时间 (秒)')
        self.ax1.set_ylabel('角度 (度)')
        self.ax1.grid(True)
        
        self.ax2.set_title('最新步态周期', fontsize=12)
        self.ax2.set_xlabel('时间 (秒)')
        self.ax2.set_ylabel('角度 (度)')
        self.ax2.grid(True)
        
        self.fig.tight_layout()
    
    def setup_plot_interactions(self):
        """设置图表交互功能（水平方向缩放和移动）"""
        # 鼠标状态
        self.pan_active = False
        self.zoom_active = False
        self.press_x = None
        self.press_y = None
        self.xlim_backup = None
        self.ylim_backup = None
        
        # 连接鼠标事件
        self.canvas.mpl_connect('button_press_event', self.on_press)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        
        # 启用鼠标移动事件
        self.canvas.mpl_connect('axes_enter_event', lambda e: self.canvas.mpl_connect('motion_notify_event', self.on_motion))
    
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
    
    def on_scroll(self, event):
        """鼠标滚轮事件（用于水平缩放）"""
        if event.inaxes is None:
            return
        
        # 获取当前X轴范围
        if event.inaxes == self.ax1:
            xlim = self.ax1.get_xlim()
            ax = self.ax1
        elif event.inaxes == self.ax2:
            xlim = self.ax2.get_xlim()
            ax = self.ax2
        else:
            return
        
        # 计算缩放中心点（鼠标位置）
        xdata = event.xdata
        if xdata is None:
            return
        
        # 计算缩放比例（向上滚动放大，向下滚动缩小）
        scale_factor = 1.1 if event.button == 'up' else 0.9
        
        # 计算新的X轴范围（以鼠标位置为中心缩放）
        x_range = xlim[1] - xlim[0]
        new_x_range = x_range * scale_factor
        
        # 计算缩放中心点相对于当前范围的位置
        center_ratio = (xdata - xlim[0]) / x_range
        
        # 计算新的范围
        new_xlim = (
            xdata - center_ratio * new_x_range,
            xdata + (1 - center_ratio) * new_x_range
        )
        
        # 应用新的X轴范围
        ax.set_xlim(new_xlim)
        
        # Y轴保持不变
        if event.inaxes == self.ax1:
            ylim = self.ax1.get_ylim()
            self.ax1.set_ylim(ylim)
        elif event.inaxes == self.ax2:
            ylim = self.ax2.get_ylim()
            self.ax2.set_ylim(ylim)
        
        self.canvas.draw_idle()
    
    def update_plots(self):
        """更新图表"""
        # 处理新数据
        self.collector.process_data()
        
        # 处理原始返回数据（记录到历史）
        raw_lines = self.collector.get_raw_data()
        for line in raw_lines:
            if line.strip():  # 只记录非空行
                self.add_history(line.strip(), "RX")
        
        # 更新采集状态显示
        if self.collector.is_collecting:
            total_points = len(self.collector.hip_data)
            queue_size = self.collector.data_queue.qsize()
            if total_points > 0:
                self.collect_status_label.config(
                    text=f"采集状态: 运行中 ({total_points} 点, 队列: {queue_size})", 
                    foreground="green"
                )
            elif queue_size > 0:
                self.collect_status_label.config(
                    text=f"采集状态: 运行中（处理中... 队列: {queue_size}）", 
                    foreground="orange"
                )
            else:
                self.collect_status_label.config(
                    text="采集状态: 运行中（等待数据...）", 
                    foreground="orange"
                )
        
        # 更新实时数据图
        # 保存当前X轴范围（如果用户已经缩放/移动）
        xlim1 = self.ax1.get_xlim() if len(self.ax1.lines) > 0 else None
        auto_scale_x1 = (xlim1 is None or xlim1 == (0.0, 1.0))  # 判断是否是默认范围
        
        self.ax1.clear()
        time_data, hip_data, ankle_data = self.collector.get_realtime_data()
        if len(time_data) > 0 and len(hip_data) > 0 and len(ankle_data) > 0 and len(time_data) == len(hip_data) == len(ankle_data):
            self.ax1.plot(time_data, hip_data, 'b-', label='髋关节', linewidth=1.5)
            self.ax1.plot(time_data, ankle_data, 'r-', label='踝关节', linewidth=1.5)
            self.ax1.set_title('实时数据（髋关节和踝关节角度）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度 (度)')
            self.ax1.grid(True)
            self.ax1.legend(loc='upper right')
            
            # 如果用户已经缩放/移动，恢复之前的X轴范围；否则自动缩放
            if not auto_scale_x1 and xlim1 is not None:
                self.ax1.set_xlim(xlim1)
            else:
                self.ax1.relim()
                self.ax1.autoscale()
        else:
            self.ax1.text(0.5, 0.5, '等待数据...', 
                         horizontalalignment='center', verticalalignment='center',
                         transform=self.ax1.transAxes, fontsize=14)
            self.ax1.set_title('实时数据（髋关节和踝关节角度）', fontsize=12)
            self.ax1.set_xlabel('时间 (秒)')
            self.ax1.set_ylabel('角度 (度)')
            self.ax1.grid(True)
        
        # 更新步态周期图
        # 性能优化：如果载入了外部数据且没有新数据，不需要频繁重绘
        cycle_time, cycle_hip, cycle_ankle = self.collector.get_gait_cycle_data()
        cycle_data_changed = (len(cycle_time) != self._last_cycle_len)
        
        # 只在数据变化或首次载入时更新
        if cycle_data_changed or not self.collector.is_loaded_data:
            # 保存当前X轴范围（如果用户已经缩放/移动）
            xlim2 = self.ax2.get_xlim() if len(self.ax2.lines) > 0 else None
            auto_scale_x2 = (xlim2 is None or xlim2 == (0.0, 1.0))  # 判断是否是默认范围
            
            self.ax2.clear()
            if len(cycle_time) > 0 and len(cycle_hip) > 0 and len(cycle_ankle) > 0:
                self.ax2.plot(cycle_time, cycle_hip, 'b-', label='髋关节', linewidth=2)
                self.ax2.plot(cycle_time, cycle_ankle, 'r-', label='踝关节', linewidth=2)
                self.ax2.set_title('最新步态周期', fontsize=12)
                self.ax2.set_xlabel('时间 (秒)')
                self.ax2.set_ylabel('角度 (度)')
                self.ax2.grid(True)
                self.ax2.legend(loc='upper right')
                
                # 如果用户已经缩放/移动，恢复之前的X轴范围；否则自动缩放
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
            
            # 记录当前周期数据长度
            self._last_cycle_len = len(cycle_time)
        
        # 绘制图表（只在需要时重绘）
        if cycle_data_changed or not self.collector.is_loaded_data:
            self.canvas.draw()
        else:
            # 对于载入的静态数据，使用draw_idle减少CPU占用
            self.canvas.draw_idle()
        
        # 定时更新
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

# ============================================================================
# 主程序
# ============================================================================

def main():
    root = tk.Tk()
    app = GaitDataCollectorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
