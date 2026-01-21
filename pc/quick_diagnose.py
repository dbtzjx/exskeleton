#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速诊断脚本：测试数据流
用法: python quick_diagnose.py COM4
"""

import sys
import json
import serial
import time
from collections import deque
from threading import Thread

class QuickDiagnose:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        
        # 数据统计
        self.total_received = 0
        self.total_parsed = 0
        self.error_count = 0
        self.last_data = None
        
        # 字段统计
        self.fields_present = {
            't': 0, 'h': 0, 'hf': 0, 'hvf': 0,
            'phase': 0, 's': 0, 'a': 0, 'ar': 0
        }
        
    def connect(self):
        """连接串口"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ 已连接到 {self.port}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False
    
    def read_loop(self):
        """读取串口数据"""
        print("\n[开始接收数据...]\n")
        start_time = time.time()
        
        while self.running:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    self.total_received += 1
                    
                    # 尝试解析JSON
                    try:
                        data = json.loads(line)
                        self.total_parsed += 1
                        self.last_data = data
                        
                        # 统计字段
                        for field in self.fields_present:
                            if field in data:
                                self.fields_present[field] += 1
                        
                        # 每接收20个数据点打印一次
                        if self.total_parsed % 20 == 0:
                            self.print_status()
                    
                    except json.JSONDecodeError:
                        self.error_count += 1
                        if self.error_count <= 3:
                            print(f"  ✗ JSON解析错误 (第{self.error_count}个): {line[:50]}")
                else:
                    time.sleep(0.01)
            
            except Exception as e:
                print(f"✗ 读取错误: {e}")
                time.sleep(0.1)
    
    def print_status(self):
        """打印当前状态"""
        elapsed = time.time() - start_time if hasattr(self, 'start_time') else 0
        data_rate = self.total_parsed / max(1, elapsed)
        
        print(f"\n[状态] 已接收 {self.total_received} 条, 成功解析 {self.total_parsed} 条 ({data_rate:.1f} Hz)")
        print(f"       JSON解析错误: {self.error_count}")
        
        if self.last_data:
            print(f"[最新数据]:")
            for field in ['t', 'h', 'hf', 'hvf', 'phase', 's', 'a', 'ar']:
                if field in self.last_data:
                    print(f"  {field}={self.last_data[field]}")
                else:
                    print(f"  {field}=<缺失> ⚠️")
        
        print(f"[字段统计] 共接收:")
        for field in ['t', 'h', 'hf', 'hvf', 'phase', 's', 'a', 'ar']:
            status = "✓" if self.fields_present[field] > 0 else "✗"
            print(f"  {status} {field}: {self.fields_present[field]}")
        print()
    
    def run(self, duration=30):
        """运行诊断，持续duration秒"""
        if not self.connect():
            return False
        
        self.running = True
        self.start_time = time.time()
        
        # 启动读取线程
        read_thread = Thread(target=self.read_loop, daemon=True)
        read_thread.start()
        
        # 等待指定时间
        try:
            print(f"\n等待 {duration} 秒接收数据...\n")
            time.sleep(duration)
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.running = False
            read_thread.join(timeout=2)
        
        # 最终统计
        self.print_summary()
        return True
    
    def print_summary(self):
        """打印总结"""
        elapsed = time.time() - self.start_time
        
        print("\n" + "="*60)
        print("[诊断结果总结]")
        print("="*60)
        
        if self.total_parsed == 0:
            print("✗ 未接收到任何有效数据")
            print("  检查项：")
            print("  1. 串口是否正确连接")
            print("  2. 固件是否在发送数据")
            print("  3. 波特率是否为 115200")
            return
        
        data_rate = self.total_parsed / elapsed
        print(f"✓ 接收到 {self.total_parsed} 个数据点，耗时 {elapsed:.1f} 秒")
        print(f"✓ 平均数据速率: {data_rate:.1f} Hz")
        
        # 检查缺失的字段
        missing_fields = []
        for field in ['t', 'h', 'hf', 'hvf', 'phase', 's', 'a', 'ar']:
            if self.fields_present[field] == 0:
                missing_fields.append(field)
        
        if missing_fields:
            print(f"\n⚠️ 缺失的字段: {', '.join(missing_fields)}")
            print("  这些字段在所有接收的数据中都没有出现")
            print("  检查固件是否发送了这些字段")
        else:
            print("\n✓ 所有字段都已接收")
        
        if self.error_count > 0:
            print(f"\n⚠️ 有 {self.error_count} 条记录的JSON解析失败")
            print("  这些可能是非JSON格式的调试输出或损坏的数据")
        
        print("\n[建议]:")
        if data_rate > 100:
            print("✓ 数据速率良好，可以开始处理")
        else:
            print("⚠️ 数据速率较低，可能是通信问题")
        
        if missing_fields:
            print(f"⚠️ 缺少字段 {missing_fields}，需要修改固件或GUI期望")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python quick_diagnose.py <串口>")
        print("示例: python quick_diagnose.py COM4")
        sys.exit(1)
    
    port = sys.argv[1]
    diagnose = QuickDiagnose(port)
    diagnose.run(duration=30)
