# 步态数据采集程序

上位机程序，用于采集、可视化和存储步态数据。

## 功能

1. **实时数据采集**：从串口读取髋关节和踝关节的实时角度数据
2. **实时可视化**：用二维图表实时显示步态曲线
3. **步态周期识别**：自动识别步态周期并存储最新一个周期
4. **数据存储**：将步态周期数据保存为JSON格式（明文）
5. **数据载入**：支持载入已保存的步态周期数据并显示

## 安装依赖

**Windows 系统：**
```bash
py -m pip install -r requirements.txt
```

**Linux/Mac 系统：**
```bash
pip install -r requirements.txt
# 或
pip3 install -r requirements.txt
```

## 使用方法

### 1. GUI版本（推荐）

**Windows 系统：**
```bash
py gait_data_collector_gui.py
```

**Linux/Mac 系统：**
```bash
python gait_data_collector_gui.py
# 或
python3 gait_data_collector_gui.py
```

**使用步骤：**
1. 运行程序，打开图形界面
2. 选择串口，点击"连接"按钮
3. 在"控制命令"输入框中输入命令，点击"发送"按钮（或按回车键）
   - 也可以点击常用命令按钮快速发送
4. 点击"开始数据采集"按钮开始采集数据
5. 图表会实时显示步态曲线
6. 程序会自动识别步态周期并保存到 `gait_cycle_data.json`

### 2. 命令行版本

**Windows 系统：**
```bash
py gait_data_collector.py
```

**Linux/Mac 系统：**
```bash
python gait_data_collector.py
# 或
python3 gait_data_collector.py
```

程序会：
1. 自动查找串口（或手动输入串口名称）
2. 连接串口
3. 启动可视化窗口（后台运行）
4. 启动数据采集线程（后台读取串口数据）
5. 显示命令输入界面，可以直接输入控制命令

**使用步骤：**
1. 运行程序，连接串口
2. 可视化窗口会自动打开并实时更新
3. 在命令提示符 `>` 后输入命令，例如：
   - 输入 `e` 使能电机
   - 输入 `gc` 启动步态数据采集
   - 输入 `gcs` 停止数据采集
   - 输入 `q` 退出程序
4. 程序会自动识别步态周期并保存到 `gait_cycle_data.json`

### 2. 数据载入模式

**Windows 系统：**
```bash
py gait_data_collector.py load
```

**Linux/Mac 系统：**
```bash
python gait_data_collector.py load
# 或
python3 gait_data_collector.py load
```

程序会：
1. 提示输入数据文件名（默认：`gait_cycle_data.json`）
2. 载入数据并显示步态曲线

## 数据格式

步态周期数据以JSON格式存储，文件名为 `gait_cycle_data.json`：

```json
{
  "timestamp": "2024-01-01T12:00:00",
  "cycle_duration": 1.2,
  "data_points": 60,
  "time": [0.0, 0.02, 0.04, ...],
  "hip_angle": [0.0, 5.2, 10.5, ...],
  "ankle_angle": [0.0, 2.1, 4.3, ...]
}
```

- `timestamp`: 数据采集时间
- `cycle_duration`: 步态周期时长（秒）
- `data_points`: 数据点数量
- `time`: 相对时间数组（秒，从0开始）
- `hip_angle`: 髋关节角度数组（度）
- `ankle_angle`: 踝关节角度数组（度）

## 步态周期识别

程序使用简单的周期识别算法：
- 基于髋关节角度回到起始位置来识别周期结束
- 允许±5度的误差
- 周期时长至少500ms

## 串口控制功能

程序集成了串口控制功能，可以直接在上位机程序中输入命令控制下位机，**无需在下位机串口输入命令**。

**支持的串口命令：**
- `gc` 或 `gaitstart` - 启动步态数据采集（默认20ms间隔）
- `gc <interval>` - 启动采集（指定间隔，ms，如：`gc 20`）
- `gcs` 或 `gaitstop` - 停止步态数据采集
- `e` 或 `enable` - 使能电机
- `d` 或 `disable` - 掉电电机
- `r` 或 `read` - 读取角度
- `s` 或 `status` - 显示状态
- `sw1 <amp>` - 电机1摆动（如：`sw1 10`）
- `sw2 <amp>` - 电机2摆动（如：`sw2 10`）
- `stop1`/`stop2` - 停止摆动
- `h` 或 `help` - 显示帮助
- `q` 或 `quit` - 退出程序

## 使用流程

1. **启动程序**：运行 `python gait_data_collector.py`
2. **连接串口**：输入串口名称（或直接回车自动查找）
3. **可视化窗口自动打开**：程序会自动启动可视化窗口，实时显示步态曲线
4. **输入控制命令**：在命令提示符 `>` 后输入命令，例如：
   - 输入 `e` 使能电机
   - 输入 `gc` 启动步态数据采集
   - 输入 `gcs` 停止数据采集
   - 输入 `q` 退出程序
5. **自动保存数据**：程序会自动识别步态周期并保存到 `gait_cycle_data.json`

## 注意事项

1. **数据采集和控制使用同一个串口**，命令和数据不会冲突
2. 程序会自动过滤下位机的调试输出，避免干扰界面
3. GUI版本中，图表会实时更新，无需手动刷新
4. 数据采集过程中，程序会自动识别并保存步态周期
5. **只保存最新的一个步态周期**（覆盖之前的）
6. GUI版本中，关闭窗口即可退出程序

## 中文显示问题

如果图表中的中文显示为乱码，程序已自动配置中文字体。如果仍有问题，可以：
1. 确保系统安装了中文字体（如：SimHei、Microsoft YaHei）
2. 在代码中修改 `plt.rcParams['font.sans-serif']` 列表，添加系统可用的中文字体
