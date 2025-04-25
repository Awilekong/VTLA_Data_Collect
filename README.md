# 机器人遥操作数据采集与转换工具

这个项目提供了一套用于机器人遥操作数据采集和格式转换的工具。主要用于利用RealSense摄像头采集机械臂操作数据，并将数据转换为特定格式(LeRobot格式)。

## 功能特点

- 多摄像头数据采集：同时采集全局视角和机械臂腕部视角的图像
- 实时机器人状态记录：记录7个关节角度和夹爪宽度
- 机器人动作捕获：记录末端执行器的位置和姿态（四元数）
- 高精度时间戳：所有数据均带有同步时间戳
- 数据格式转换：将采集的HDF5格式数据转换为LeRobot格式
- 支持上传至Hugging Face Hub：方便数据共享和协作

## 系统要求

- Python 3.8+
- RealSense摄像头 (两个，用于全局和腕部视角)
- 支持的机器人：Franka Emika Panda/Research3 (通过Polymetis控制)

## 安装方法

1. 克隆此仓库：
```bash
git clone https://github.com/yourusername/collect_demo.git
cd collect_demo
```

2. 安装依赖项：
```bash
pip install -r requirements.txt
```

如果requirements.txt不存在，请安装以下包：
```bash
pip install numpy opencv-python pyrealsense2 h5py readchar pynput polymetis tyro tensorflow tensorflow_datasets
```

## 使用方法

### 查看RealSense摄像头序列号

```bash
python camera.py
```

### 数据采集

1. 修改`collect_demo.py`中的摄像头序列号和保存目录：
```python
self.serial_1 = "你的全局摄像头序列号"
self.serial_2 = "你的腕部摄像头序列号"
self.save_dir = "collected_data_right"  # 数据保存目录
```

2. 启动数据采集：
```bash
python collect_demo.py
```

3. 按Enter键开始采集，按ESC键停止采集。

### 查看采集的数据

```bash
python viewh5.py
```
注意：请先修改viewh5.py中的文件路径为你的实际数据文件路径。

### 转换数据格式

将采集的数据转换为LeRobot格式：
```bash
python convert_my_data_to_lerobot.py --data_dir 你的数据目录 --task_description "任务描述"
```

可选：将数据上传至Hugging Face Hub：
```bash
python convert_my_data_to_lerobot.py --data_dir 你的数据目录 --push_to_hub
```

## 数据格式

采集的HDF5数据包含以下内容：
- `observation.images.image`: 全局视角RGB图像 (256x256x3)
- `observation.images.wrist_image`: 腕部视角RGB图像 (256x256x3)
- `observation.state`: 机器人状态 (8维：7个关节角度 + 1个夹爪宽度)
- `action`: 机器人动作 (7维：3个位置 + 4个四元数)
- `timestamps`: 时间戳

## 项目结构

- `collect_demo.py`: 主要的数据采集脚本
- `convert_my_data_to_lerobot.py`: 数据格式转换脚本
- `viewh5.py`: 查看采集的HDF5数据
- `camera.py`: 检测RealSense摄像头序列号
- `collected_data_*/`: 存放采集的数据

## 注意事项

1. 使用前请确认RealSense摄像头已正确连接并安装驱动
2. 确保机器人IP地址配置正确
3. 采集频率默认为10Hz，可在初始化DataCollector时修改

## 许可证

MIT License 