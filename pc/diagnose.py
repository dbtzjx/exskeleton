#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机诊断工具
用于快速诊断曲线不显示的问题
"""

import sys
import os

def check_imports():
    """检查必要的库是否安装"""
    print("\n" + "="*60)
    print("1. 检查依赖库")
    print("="*60)
    
    required_libs = {
        'serial': 'pyserial',
        'matplotlib': 'matplotlib',
        'tkinter': 'tkinter',  # 通常内置
        'numpy': 'numpy',
        'json': 'json',  # 标准库
    }
    
    missing = []
    for lib, name in required_libs.items():
        try:
            __import__(lib)
            print(f"✅ {name}")
        except ImportError:
            print(f"❌ {name} (缺失)")
            missing.append(name)
    
    if missing:
        print(f"\n⚠️  缺失库: {', '.join(missing)}")
        print("修复方法:")
        print("  pip install pyserial matplotlib numpy")
        return False
    
    print("\n✓ 所有依赖库已安装")
    return True

def check_serial_ports():
    """检查可用的串口"""
    print("\n" + "="*60)
    print("2. 检查串口")
    print("="*60)
    
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("❌ 未找到任何串口")
            print("请检查:")
            print("  1. USB 连接线是否接好")
            print("  2. 设备驱动程序是否已安装")
            print("  3. 设备是否已打开电源")
            return False
        
        print(f"✅ 找到 {len(ports)} 个串口:")
        for i, port in enumerate(ports):
            print(f"   {i+1}. {port.device} - {port.description}")
        
        return True
    except Exception as e:
        print(f"❌ 检查串口失败: {e}")
        return False

def check_json_format():
    """验证 JSON 数据格式"""
    print("\n" + "="*60)
    print("3. 检查 JSON 格式")
    print("="*60)
    
    import json
    
    # 示例数据
    examples = [
        ('正常 (仅必要字段)', '{"t": 1234, "h": 15.5}'),
        ('完整 (推荐)', '{"t": 1234, "h": 15.5, "hf": 15.2, "hvf": 25.0, "phase": 1, "s": 0.5, "a": 9.0, "ar": 8.5}'),
        ('不完整', '{"t": 1234}'),
        ('无效', '{t: 1234, h: 15.5}'),
    ]
    
    for desc, json_str in examples:
        try:
            data = json.loads(json_str)
            has_required = 't' in data and 'h' in data
            status = "✅" if has_required else "⚠️"
            print(f"{status} {desc}")
            print(f"   输入: {json_str}")
            print(f"   必要字段: {'t' in data}(t), {'h' in data}(h)")
        except json.JSONDecodeError as e:
            print(f"❌ {desc}")
            print(f"   错误: {e}")

def check_code_structure():
    """检查代码结构"""
    print("\n" + "="*60)
    print("4. 检查代码结构")
    print("="*60)
    
    file = "gait_data_collector_gui.py"
    
    if not os.path.exists(file):
        print(f"❌ 找不到文件: {file}")
        return False
    
    print(f"✅ 文件存在: {file}")
    
    # 检查关键类和方法
    try:
        with open(file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        checks = [
            ('class GaitDataCollector', '数据收集器类'),
            ('class GaitDataCollectorGUI', 'GUI 类'),
            ('def _hip_process_loop', '髋关节数据处理'),
            ('def _gait_process_loop', '步态采集处理'),
            ('def update_plots', '图表更新'),
        ]
        
        for pattern, desc in checks:
            if pattern in content:
                print(f"✅ {desc}")
            else:
                print(f"❌ {desc} (缺失)")
                return False
        
        return True
    except Exception as e:
        print(f"❌ 检查失败: {e}")
        return False

def print_usage_tips():
    """打印使用建议"""
    print("\n" + "="*60)
    print("5. 使用建议")
    print("="*60)
    
    tips = [
        "1. 连接串口后，点击 '髋关节数据' 或 '步态采集' 启用数据处理",
        "2. 在终端中查看是否有 '[模块初始化]' 日志",
        "3. 确保固件发送的 JSON 包含 't' 和 'h' 字段",
        "4. 如果仍无数据，检查 '队列大小' 是否在增加",
        "5. 查看详细诊断指南: 曲线无法显示-故障排查.md",
    ]
    
    for tip in tips:
        print(f"   {tip}")

def main():
    """主诊断程序"""
    print("\n")
    print("="*60)
    print("     上位机诊断工具 v1.0")
    print("="*60)
    
    checks = [
        ("依赖库检查", check_imports),
        ("串口检查", check_serial_ports),
        ("JSON 格式检查", check_json_format),
        ("代码结构检查", check_code_structure),
    ]
    
    results = []
    for name, check_func in checks:
        try:
            result = check_func()
            results.append((name, result))
        except Exception as e:
            print(f"❌ {name} 失败: {e}")
            results.append((name, False))
    
    # 总结
    print("\n" + "="*60)
    print("诊断总结")
    print("="*60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✅ 通过" if result else "❌ 未通过"
        print(f"{status} - {name}")
    
    print(f"\n总体: {passed}/{total} 项检查通过")
    
    if passed == total:
        print("\n✅ 诊断成功！环境配置正确。")
        print("\n后续步骤:")
        print("  1. 运行: python gait_data_collector_gui.py")
        print("  2. 连接串口")
        print("  3. 点击 '髋关节数据' 启用模块")
        print("  4. 查看图表是否显示曲线")
    else:
        print("\n❌ 诊断发现问题。请根据上面的信息进行修复。")
        print_usage_tips()
    
    print("\n" + "="*60 + "\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️  用户中断诊断")
    except Exception as e:
        print(f"\n❌ 诊断异常: {e}")
        import traceback
        traceback.print_exc()
