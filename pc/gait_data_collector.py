#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步态数据采集与可视化程序
功能：
1. 读取串口数据（髋关节、踝关节角度）
2. 实时绘制步态曲线
3. 识别步态周期并存储最新一个周期
4. 支持数据载入和显示
"""

import serial
import serial.tools.list_ports
import json
import time
import threading
import queue
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
from datetime import datetime
import os
import sys

# ============================================================================
# 配置参数
# ============================================================================

SERIAL_BAUDRATE = 115200
MAX_DATA_POINTS = 2000  # 最大数据点数（用于实时显示）
GAIT_CYCLE_FILE = "gait_cycle_data.json"  # 步态周期数据文件

# ============================================================================
# 数据采集类
# ============================================================================

class GaitDataCollector:
    def __init__(self):
        self.serial_port = None
        self.data_queue = queue.Queue()
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
        
        # 步态周期识别
        self.last_hip_angle = None
        self.cycle_start_time = None
        self.cycle_start_hip = None
        self.in_cycle = False
        
    def find_serial_port(self):
        """自动查找串口"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # 优先查找包含 "Teensy" 或 "USB" 的串口
            if "Teensy" in port.description or "USB" in port.description:
                return port.device
        # 如果没找到，返回第一个可用串口
        if ports:
            return ports[0].device
        return None
    
    def connect_serial(self, port=None, baudrate=115200):
        """连接串口"""
        if port is None:
            port = self.find_serial_port()
        
        if port is None:
            print("错误：未找到可用串口")
            return False
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            print(f"已连接到串口: {port} ({baudrate} baud)")
            # 清空串口缓冲区
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            return True
        except Exception as e:
            print(f"连接串口失败: {e}")
            return False
    
    def send_command(self, command):
        """发送命令到串口"""
        if self.serial_port is None or not self.serial_port.is_open:
            print("错误：串口未连接")
            return False
        
        try:
            command_with_newline = command + '\n'
            self.serial_port.write(command_with_newline.encode('utf-8'))
            self.serial_port.flush()
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def disconnect_serial(self):
        """断开串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已断开")
    
    def start_collection(self):
        """启动数据采集"""
        if self.serial_port is None or not self.serial_port.is_open:
            print("错误：串口未连接")
            return False
        
        if self.is_collecting:
            print("数据采集已在进行中")
            return False
        
        self.is_collecting = True
        self.collect_thread = threading.Thread(target=self._collect_data, daemon=True)
        self.collect_thread.start()
        print("数据采集已启动")
        return True
    
    def stop_collection(self):
        """停止数据采集"""
        self.is_collecting = False
        if self.collect_thread:
            self.collect_thread.join(timeout=2)
        print("数据采集已停止")
    
    def _collect_data(self):
        """数据采集线程（从串口读取数据）"""
        buffer = ""
        while self.is_collecting:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 按行处理数据
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        # 跳过空行
                        if not line:
                            continue
                        
                        # 跳过命令提示行（避免干扰）
                        if line.startswith('>') or line.startswith('Command:'):
                            continue
                        
                        # 解析JSON数据（步态数据）
                        if line.startswith('{') and line.endswith('}'):
                            try:
                                data_dict = json.loads(line)
                                if 't' in data_dict and 'h' in data_dict and 'a' in data_dict:
                                    self.data_queue.put(data_dict)
                            except json.JSONDecodeError:
                                pass
                        # 其他输出（下位机的调试信息）不打印到控制台，避免干扰命令输入界面
                        # 如果需要查看下位机输出，可以取消注释下面一行
                        # else:
                        #     print(f"[下位机] {line}")
                
                time.sleep(0.01)  # 避免CPU占用过高
            except Exception as e:
                print(f"数据采集错误: {e}")
                break
    
    def process_data(self):
        """处理接收到的数据（在主线程中调用）"""
        while not self.data_queue.empty():
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
                
            except queue.Empty:
                break
    
    def _detect_gait_cycle(self, timestamp, hip_angle, ankle_angle):
        """检测步态周期（简单方法：基于髋关节回到起始位置）"""
        if self.last_hip_angle is None:
            self.last_hip_angle = hip_angle
            self.cycle_start_time = timestamp
            self.cycle_start_hip = hip_angle
            self.in_cycle = True
            # 重置周期数据
            self.gait_cycle_time = []
            self.gait_cycle_hip = []
            self.gait_cycle_ankle = []
            return
        
        # 检测是否完成一个周期（髋关节回到起始位置附近，允许±5度误差）
        if self.in_cycle:
            # 添加到当前周期
            self.gait_cycle_time.append(timestamp)
            self.gait_cycle_hip.append(hip_angle)
            self.gait_cycle_ankle.append(ankle_angle)
            
            # 检测周期结束（髋关节回到起始位置，且已经过了一段时间）
            angle_diff = abs(hip_angle - self.cycle_start_hip)
            time_diff = timestamp - self.cycle_start_time
            
            if angle_diff < 5.0 and time_diff > 500:  # 回到起始位置且至少500ms
                # 周期完成，保存数据
                self._save_gait_cycle()
                # 开始新周期
                self.cycle_start_time = timestamp
                self.cycle_start_hip = hip_angle
                self.gait_cycle_time = [timestamp]
                self.gait_cycle_hip = [hip_angle]
                self.gait_cycle_ankle = [ankle_angle]
        else:
            self.in_cycle = True
            self.cycle_start_time = timestamp
            self.cycle_start_hip = hip_angle
            self.gait_cycle_time = [timestamp]
            self.gait_cycle_hip = [hip_angle]
            self.gait_cycle_ankle = [ankle_angle]
        
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
                with open(GAIT_CYCLE_FILE, 'w', encoding='utf-8') as f:
                    json.dump(gait_data, f, indent=2, ensure_ascii=False)
                print(f"\n步态周期已保存: {len(self.gait_cycle_time)} 个数据点, "
                      f"周期时长: {gait_data['cycle_duration']:.2f} 秒")
            except Exception as e:
                print(f"保存步态周期数据失败: {e}")
    
    def load_gait_cycle(self, filename=None):
        """载入步态周期数据"""
        if filename is None:
            filename = GAIT_CYCLE_FILE
        
        if not os.path.exists(filename):
            print(f"文件不存在: {filename}")
            return False
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self.gait_cycle_time = data.get('time', [])
            self.gait_cycle_hip = data.get('hip_angle', [])
            self.gait_cycle_ankle = data.get('ankle_angle', [])
            
            print(f"已载入步态周期数据: {len(self.gait_cycle_time)} 个数据点")
            if 'cycle_duration' in data:
                print(f"周期时长: {data['cycle_duration']:.2f} 秒")
            if 'timestamp' in data:
                print(f"采集时间: {data['timestamp']}")
            
            return True
        except Exception as e:
            print(f"载入步态周期数据失败: {e}")
            return False
    
    def get_realtime_data(self):
        """获取实时数据（用于绘图）"""
        if len(self.time_data) == 0:
            return [], [], []
        
        # 转换为相对时间（从最新数据往前）
        if len(self.time_data) > 0:
            latest_time = self.time_data[-1]
            relative_time = [(t - latest_time) / 1000.0 for t in self.time_data]  # 转换为秒
            return list(relative_time), list(self.hip_data), list(self.ankle_data)
        return [], [], []
    
    def get_gait_cycle_data(self):
        """获取步态周期数据（用于绘图）"""
        return (list(self.gait_cycle_time),
                list(self.gait_cycle_hip),
                list(self.gait_cycle_ankle))

# ============================================================================
# 可视化类
# ============================================================================

class GaitVisualizer:
    def __init__(self, collector):
        self.collector = collector
        self.fig = None
        self.ax1 = None  # 实时数据图
        self.ax2 = None  # 步态周期图
        self.ani = None
        self.command_thread = None
        self.command_running = False
        
    def setup_plots(self):
        """设置绘图"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('步态数据采集与可视化', fontsize=14, fontweight='bold')
        
        # 实时数据图
        self.ax1.set_title('实时数据（髋关节和踝关节角度）')
        self.ax1.set_xlabel('时间 (秒)')
        self.ax1.set_ylabel('角度 (度)')
        self.ax1.grid(True)
        self.ax1.legend(['髋关节', '踝关节'], loc='upper right')
        
        # 步态周期图
        self.ax2.set_title('最新步态周期')
        self.ax2.set_xlabel('时间 (秒)')
        self.ax2.set_ylabel('角度 (度)')
        self.ax2.grid(True)
        self.ax2.legend(['髋关节', '踝关节'], loc='upper right')
        
        plt.tight_layout()
    
    def update_plots(self, frame):
        """更新绘图（动画回调）"""
        try:
            # 处理新数据
            self.collector.process_data()
            
            # 更新实时数据图
            self.ax1.clear()
            time_data, hip_data, ankle_data = self.collector.get_realtime_data()
            if len(time_data) > 0 and len(hip_data) > 0 and len(ankle_data) > 0:
                self.ax1.plot(time_data, hip_data, 'b-', label='髋关节', linewidth=1.5)
                self.ax1.plot(time_data, ankle_data, 'r-', label='踝关节', linewidth=1.5)
                self.ax1.set_title('实时数据（髋关节和踝关节角度）')
                self.ax1.set_xlabel('时间 (秒)')
                self.ax1.set_ylabel('角度 (度)')
                self.ax1.grid(True)
                self.ax1.legend(loc='upper right')
            
            # 更新步态周期图
            self.ax2.clear()
            cycle_time, cycle_hip, cycle_ankle = self.collector.get_gait_cycle_data()
            if len(cycle_time) > 0 and len(cycle_hip) > 0 and len(cycle_ankle) > 0:
                self.ax2.plot(cycle_time, cycle_hip, 'b-', label='髋关节', linewidth=2)
                self.ax2.plot(cycle_time, cycle_ankle, 'r-', label='踝关节', linewidth=2)
                self.ax2.set_title('最新步态周期')
                self.ax2.set_xlabel('时间 (秒)')
                self.ax2.set_ylabel('角度 (度)')
                self.ax2.grid(True)
                self.ax2.legend(loc='upper right')
            else:
                self.ax2.text(0.5, 0.5, '暂无步态周期数据\n（等待数据采集...）', 
                             horizontalalignment='center', verticalalignment='center',
                             transform=self.ax2.transAxes, fontsize=12)
                self.ax2.set_title('最新步态周期')
                self.ax2.set_xlabel('时间 (秒)')
                self.ax2.set_ylabel('角度 (度)')
                self.ax2.grid(True)
            
            # 刷新图形（非阻塞模式）
            plt.draw()
            plt.pause(0.001)
            
        except Exception as e:
            # 忽略绘图错误，避免影响主程序
            pass
    
    def _command_input_thread(self):
        """命令输入线程"""
        self.command_running = True
        print("\n" + "="*60)
        print("串口控制命令（在下方输入命令，按回车发送）")
        print("常用命令：")
        print("  gc 或 gaitstart     - 启动步态数据采集")
        print("  gc <interval>       - 启动采集（指定间隔，ms，如：gc 20）")
        print("  gcs 或 gaitstop     - 停止步态数据采集")
        print("  e 或 enable         - 使能电机")
        print("  d 或 disable        - 掉电电机")
        print("  r 或 read           - 读取角度")
        print("  s 或 status         - 显示状态")
        print("  h 或 help           - 显示帮助")
        print("  sw1 <amp>           - 电机1摆动（如：sw1 10）")
        print("  sw2 <amp>           - 电机2摆动（如：sw2 10）")
        print("="*60 + "\n")
        
        while self.command_running:
            try:
                # 使用 input() 在非主线程中可能会出错，改用从标准输入读取
                # 这里我们需要在主线程中处理输入，所以使用队列
                time.sleep(0.1)
            except:
                break
    
    def start_animation(self):
        """启动动画（非阻塞模式）"""
        if self.ani is None:
            self.setup_plots()
            self.ani = animation.FuncAnimation(self.fig, self.update_plots, 
                                              interval=50, blit=False)
        # 使用非阻塞模式显示窗口
        plt.ion()  # 开启交互模式
        plt.show(block=False)  # 非阻塞显示
        
    def stop_command_input(self):
        """停止命令输入"""
        self.command_running = False
    
    def show_gait_cycle(self):
        """显示载入的步态周期数据"""
        cycle_time, cycle_hip, cycle_ankle = self.collector.get_gait_cycle_data()
        if len(cycle_time) == 0:
            print("没有步态周期数据可显示")
            return
        
        plt.figure(figsize=(10, 6))
        plt.plot(cycle_time, cycle_hip, 'b-', label='髋关节', linewidth=2)
        plt.plot(cycle_time, cycle_ankle, 'r-', label='踝关节', linewidth=2)
        plt.title('步态周期数据')
        plt.xlabel('时间 (秒)')
        plt.ylabel('角度 (度)')
        plt.grid(True)
        plt.legend(loc='upper right')
        plt.tight_layout()
        plt.show()

# ============================================================================
# 主程序
# ============================================================================

def main():
    print("=" * 60)
    print("步态数据采集与可视化程序")
    print("=" * 60)
    
    collector = GaitDataCollector()
    visualizer = GaitVisualizer(collector)
    
    # 连接串口
    port = input("\n请输入串口名称（直接回车自动查找）: ").strip()
    if not port:
        port = None
    
    if not collector.connect_serial(port, SERIAL_BAUDRATE):
        print("无法连接串口，程序退出")
        return
    
    try:
        # 启动可视化（非阻塞）
        print("\n正在启动可视化窗口...")
        visualizer.start_animation()
        time.sleep(0.5)  # 等待窗口初始化
        
        # 启动数据采集线程（后台运行，读取串口数据）
        collector.start_collection()
        
        # 命令输入循环（主线程）
        print("\n" + "="*60)
        print("串口控制命令（输入命令，按回车发送）")
        print("常用命令：")
        print("  gc 或 gaitstart     - 启动步态数据采集")
        print("  gc <interval>       - 启动采集（指定间隔，ms，如：gc 20）")
        print("  gcs 或 gaitstop     - 停止步态数据采集")
        print("  e 或 enable         - 使能电机")
        print("  d 或 disable        - 掉电电机")
        print("  r 或 read           - 读取角度")
        print("  s 或 status         - 显示状态")
        print("  sw1 <amp>           - 电机1摆动（如：sw1 10）")
        print("  sw2 <amp>           - 电机2摆动（如：sw2 10）")
        print("  stop1/stop2         - 停止摆动")
        print("  h 或 help           - 显示帮助")
        print("  q 或 quit           - 退出程序")
        print("="*60 + "\n")
        print("提示：可视化窗口在后台更新，关闭窗口或输入 'q' 可退出程序\n")
        
        # 命令输入循环（主线程）
        # 注意：matplotlib的FuncAnimation会在后台自动更新，input()阻塞不会影响图表更新
        while True:
            try:
                # 检查窗口是否还存在
                if visualizer.fig:
                    try:
                        if not plt.fignum_exists(visualizer.fig.number):
                            print("\n可视化窗口已关闭，退出程序...")
                            break
                    except:
                        # 窗口可能已经被关闭
                        break
                
                # 等待用户输入命令（会阻塞，但FuncAnimation会在后台继续更新图表）
                command = input("> ").strip()
                
                if not command:
                    continue
                
                # 处理退出命令
                if command.lower() in ['q', 'quit', 'exit']:
                    print("正在退出...")
                    break
                
                # 发送命令到串口
                if collector.send_command(command):
                    # 对于某些命令，打印确认信息
                    cmd_lower = command.lower()
                    if 'gc' in cmd_lower and 'stop' not in cmd_lower:
                        print(f"✓ 已发送采集命令: {command}")
                    elif 'stop' in cmd_lower or cmd_lower == 'gcs':
                        print(f"✓ 已发送停止命令: {command}")
                    elif cmd_lower in ['e', 'enable']:
                        print(f"✓ 已发送使能命令")
                    elif cmd_lower in ['d', 'disable']:
                        print(f"✓ 已发送掉电命令")
                    # 短暂延迟，确保命令发送完成
                    time.sleep(0.05)
                
            except EOFError:
                # Ctrl+D 退出
                break
            except KeyboardInterrupt:
                # Ctrl+C 退出
                print("\n正在退出...")
                break
            except Exception as e:
                print(f"命令处理错误: {e}")
                # 不打印完整traceback，避免干扰界面
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 停止数据采集
        collector.stop_collection()
        visualizer.stop_command_input()
        collector.disconnect_serial()
        
        # 关闭matplotlib窗口
        if visualizer.fig:
            try:
                plt.close(visualizer.fig)
            except:
                pass
        
        print("程序已退出")

def load_and_show():
    """载入并显示步态周期数据"""
    collector = GaitDataCollector()
    visualizer = GaitVisualizer(collector)
    
    filename = input("请输入数据文件名（直接回车使用默认文件）: ").strip()
    if not filename:
        filename = None
    
    if collector.load_gait_cycle(filename):
        visualizer.show_gait_cycle()
    else:
        print("无法载入数据")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "load":
        load_and_show()
    else:
        main()
