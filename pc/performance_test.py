#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机性能测试脚本
用于验证曲线绘制优化的有效性
"""

import psutil
import time
import os
import sys

class PerformanceMonitor:
    """性能监测工具"""
    
    def __init__(self, process_name="python"):
        self.process_name = process_name
        self.process = None
        self.start_time = time.time()
        self.samples = []
        
    def find_process(self):
        """查找进程"""
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if 'gait_data_collector_gui' in proc.cmdline() or proc.name() == self.process_name:
                    self.process = proc
                    return proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return None
    
    def sample(self):
        """获取一次性能样本"""
        if not self.process:
            return None
        
        try:
            cpu_percent = self.process.cpu_percent(interval=0.1)
            mem_info = self.process.memory_info()
            mem_mb = mem_info.rss / 1024 / 1024
            
            elapsed = time.time() - self.start_time
            sample = {
                'time': elapsed,
                'cpu': cpu_percent,
                'memory': mem_mb
            }
            self.samples.append(sample)
            return sample
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return None
    
    def report(self):
        """生成性能报告"""
        if not self.samples:
            print("❌ 无数据样本")
            return
        
        cpu_values = [s['cpu'] for s in self.samples]
        mem_values = [s['memory'] for s in self.samples]
        
        print("\n" + "="*60)
        print("性能监测报告")
        print("="*60)
        print(f"监测时长: {self.samples[-1]['time']:.1f} 秒")
        print(f"样本数: {len(self.samples)}")
        print()
        print("CPU 占用率:")
        print(f"  平均值: {sum(cpu_values)/len(cpu_values):.1f}%")
        print(f"  最大值: {max(cpu_values):.1f}%")
        print(f"  最小值: {min(cpu_values):.1f}%")
        print()
        print("内存占用:")
        print(f"  初始: {mem_values[0]:.1f} MB")
        print(f"  当前: {mem_values[-1]:.1f} MB")
        print(f"  增长: {mem_values[-1] - mem_values[0]:.1f} MB")
        print(f"  最大值: {max(mem_values):.1f} MB")
        
        # 判断是否通过测试
        print()
        print("测试结论:")
        if mem_values[-1] - mem_values[0] < 50:  # 内存增长 < 50MB
            print("  ✅ 内存管理：通过 (增长 < 50MB)")
        else:
            print(f"  ❌ 内存管理：未通过 (增长 > 50MB)")
        
        if sum(cpu_values)/len(cpu_values) < 20:  # 平均 CPU < 20%
            print("  ✅ CPU 占用：通过 (平均 < 20%)")
        else:
            print(f"  ❌ CPU 占用：未通过 (平均 > 20%)")
        
        print("="*60 + "\n")
    
    def start_monitoring(self, duration=600, interval=1):
        """持续监测指定时间"""
        print(f"开始监测... (持续 {duration} 秒)")
        print("提示: 请在另一个终端启动 gait_data_collector_gui.py")
        print()
        
        # 等待找到进程
        for i in range(30):
            if self.find_process():
                print(f"✅ 找到进程 (PID: {self.process.pid})")
                break
            print(f"等待进程启动... ({i+1}/30)")
            time.sleep(1)
        
        if not self.process:
            print("❌ 未找到进程")
            return
        
        # 开始采样
        start = time.time()
        while time.time() - start < duration:
            sample = self.sample()
            if sample:
                print(f"[{sample['time']:6.1f}s] CPU: {sample['cpu']:5.1f}% | "
                      f"内存: {sample['memory']:7.1f} MB")
            time.sleep(interval)
        
        # 生成报告
        self.report()

def main():
    """主程序"""
    print("=" * 60)
    print("上位机性能测试 - 曲线绘制优化验证")
    print("=" * 60)
    print()
    print("使用方法:")
    print("  1. 打开一个终端，运行: python gait_data_collector_gui.py")
    print("  2. 打开另一个终端，运行此脚本: python performance_test.py")
    print()
    print("测试将持续 10 分钟，监测内存和 CPU 占用")
    print()
    
    monitor = PerformanceMonitor()
    monitor.start_monitoring(duration=600, interval=5)  # 10 分钟

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️  用户中断测试")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
