
# 📊 VTLA数据采集系统

## 🔍 项目概述

FR3机器人触觉数据采集系统是一个用于采集、记录和可视化Franka机器人操作过程中的多模态数据的工具包，特别关注触觉数据的采集。该系统集成了GelSight Mini触觉传感器、RealSense相机和FR3机器人，可以同步采集以下数据：

- 🖐️ GelSight Mini触觉传感器数据（可选择：原始图像、tracker motion、3D点云）
- 📷 RealSense相机图像（全局视角和机器人腕部视角）
- 🤖 机器人状态（关节角度、desired action（取自gello遥操设备映射）、夹爪状态）

## 🛠️ 安装指南

### 系统要求

- Ubuntu 20.04或更高版本
- Python 3.8或更高版本
- FR3机械臂
- GelSight Mini触觉传感器（2个）
- Intel RealSense相机（2个）

### 安装步骤

1. 克隆仓库：

```bash
git clone https://github.com/yourusername/FR3-Tactile-Data-Collection.git
cd FR3-Tactile-Data-Collection
```

2. 安装依赖项：

```bash
pip install -r requirements.txt
```

3. 安装额外依赖（根据requirements.txt内容）：

```bash
pip install numpy opencv-python h5py pyrealsense2 readchar pynput
```

4. 确保GelSight SDK位于正确位置（上级目录中的gsrobotics文件夹）。

## 📁 项目结构

```
Data_Collect_FR3/
├── 📄 collect_show_tracker.py      # 数据采集和触觉显示脚本（显示2D形变场）
├── 📄 collect_show_tactile.py      # 数据采集和触觉显示脚本（显示原始触觉图像）
├── 📄 collect_show_tracker_write.py # 数据采集和触觉显示脚本（显示2D形变场并写入）
├── 📄 collect_demo.py              # 演示采集脚本（未接入gelsight）
├── 📄 gel_test.py                  # GelSight传感器测试脚本
├── 📁 gelmini/                     # GelSight Mini传感器接口代码
├── 📁 realfrank/                   # FR3机械臂控制接口代码
├── 📁 utils/                       # 辅助工具和函数
├── 📁 temp/                        # 临时文件和测试数据
├── 📄 requirements.txt             # 项目依赖项
└── 📄 README.md                    # 项目文档
```

## 📝 使用指南

### 运行数据采集程序

主要数据采集程序是`collect_show_tracker.py`，它提供完整的数据采集功能：

```bash
python collect_show_tracker.py
```

启动后，程序会提示您选择：
- 是否保存触觉点云数据
- 是否实时显示触觉形变场
- 是否保存触觉图像视频

### 键盘控制

系统使用简单的键盘控制：
- **空格键**：开始新的数据采集
- **回车键**：结束当前数据采集
- **Ctrl+C**：完全退出采集程序

### 数据格式

采集的数据以HDF5格式保存在指定目录（默认为`./collected_data`）中，文件命名格式为：
`pick_data_YYYYMMDD_HHMMSS_XXXframes.h5`

## 🧠 主要功能模块

### 🖐️ 触觉数据采集

- **基础数据采集**：记录GelSight Mini传感器的原始图像
- **形变场跟踪**：可视化GelSight传感器表面的接触点和形变
- **3D点云重建**：基于神经网络模型将触觉图像转换为3D点云

### 📸 视觉数据采集

- **全局视图**：采集机器人工作空间的全局RGB图像
- **腕部视图**：采集机器人末端执行器附近的局部RGB图像

### 🤖 机器人状态采集

- **关节状态**：记录机器人7个关节的角度和夹爪宽度
- **末端位姿**：记录末端执行器的位置和方向（四元数）
- **desired action**：7个关节角与夹爪宽度

## 📦 数据存储

系统可以记录以下数据：

| 数据类型 | 描述 | 数据格式 |
|----------|------|----------|
| `observation.images.image` | 全局相机RGB图像 | (N, 256, 256, 3) uint8 |
| `observation.images.wrist_image` | 腕部相机RGB图像 | (N, 256, 256, 3) uint8 |
| `observation.state` | 机器人关节角度和夹爪宽度 | (N, 8) float32 |
| `observation.ee_pose` | 末端执行器位姿（位置+四元数） | (N, 7) float32 |
| `action` | 机器人期望动作 | (N, 8) float32 |
| `observation.tactile.img1` | 第一个GelSight的图像 | (N, H, W, 3) uint8 |
| `observation.tactile.img2` | 第二个GelSight的图像 | (N, H, W, 3) uint8 |
| `observation.tactile.pc1` | 第一个GelSight的点云数据 | (N, P, 3) float32 |
| `observation.tactile.pc2` | 第二个GelSight的点云数据 | (N, P, 3) float32 |
| `timestamps` | 每帧数据的时间戳 | (N,) float64 |
| `observation.tactile.df1` | 第一个GelSight的tracker motion | (N*M, 2) float32 |
| `observation.tactile.df2` | 第二个GelSight的tracker motion | (N*M, 2) float32 |

## 🔌 硬件连接指南

### GelSight传感器

系统支持同时连接两个GelSight Mini传感器：
注意：以重构设备连接代码，传入0,1即可
- 传感器1：使用ID "0"
- 传感器2：使用ID "1"

如果传感器无法识别，可能需要检查USB连接或使用`v4l2-ctl --list-devices`命令检查设备ID。

### RealSense相机

系统使用序列号区分两个RealSense相机：
- 全局相机：序列号 "136622074722"
- 腕部相机：序列号 "233622071355"

请根据实际硬件更改代码中的序列号。

## 🧩 依赖项

项目主要依赖以下库：
- `numpy`：数据处理
- `opencv-python`：图像处理
- `h5py`：数据存储
- `pyrealsense2`：RealSense相机接口
- `polymetis`：FR3机器人控制接口
- `readchar`/`pynput`：键盘输入处理
- `gelmini`：GelSight Mini传感器接口（本地模块）

完整依赖项请参考`requirements.txt`。

## ⚠️ 常见问题

1. **GelSight传感器无法连接**：
   - 确保传感器正确插入USB端口
   - 检查设备权限：`sudo chmod 666 /dev/video*`
   - 确保没有其他程序占用相机

2. **机器人连接失败**：
   - 检查`robot_ip`和`gripper_ip`是否正确设置
   - 确保机器人控制系统已启动

3. **数据采集速度慢**：
   - 降低采集频率（默认10Hz）
   - 关闭点云重建功能
   - 使用较低分辨率设置

## 📧 联系方式

如有任何问题或建议，请联系项目维护者：

- **开发者**：Pengwei Zhang
- **邮箱**：[your-email@example.com]
- **实验室**：[Your Lab Name]

## 📜 许可证

本项目基于MIT许可证开源。详情请见LICENSE文件。
