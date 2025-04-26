#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) Pengwei Zhang -- 2025.4.25
# Author: Pengwei Zhang
# License: MIT

"""
脚本用于将collect_demo.py收集的HDF5格式数据转换为LeRobot格式。

使用方法:
python convert_libero_data_to_lerobot.py --data_dir /path/to/your/collected_data

如果想要将数据集上传到Hugging Face Hub，可以使用以下命令:
python convert_libero_data_to_lerobot.py --data_dir /path/to/your/collected_data --push_to_hub

注意: 运行此脚本需要安装h5py和numpy:
`pip install h5py numpy tensorflow tensorflow_datasets`

转换后的数据集将保存到$LEROBOT_HOME目录中。
"""

import shutil
import os
import h5py
import numpy as np
from pathlib import Path

# from lerobot.common.datasets.lerobot_dataset import LEROBOT_HOME
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import tyro

REPO_NAME = "Lerobot/franka_pick_center"  # 输出数据集的名称，也用于Hugging Face Hub

task_description = "Pick up the tissue box located in the center of the table."
data_dir = "collected_data_center"

def main(data_dir: str = data_dir, *, push_to_hub: bool = False, task_description: str = task_description):
    # 清理输出目录中已存在的数据集
    output_path = Path(REPO_NAME)
    try:
        if output_path.exists():
            shutil.rmtree(output_path)
    except FileExistsError:
        print(f"Warning: Unable to remove existing directory {output_path}. Please check permissions or remove it manually.")
        return

    # 创建LeRobot数据集，定义要存储的特征
    # 根据collect_demo.py中的数据格式定义特征
    dataset = LeRobotDataset.create(
        repo_id=REPO_NAME,
        robot_type="franka",
        fps=10,  # 与collect_demo.py中相同的采集频率
        features={
            "image": {
                "dtype": "image",
                "shape": (256, 256, 3),
                "names": ["height", "width", "channel"],
            },
            "wrist_image": {
                "dtype": "image",
                "shape": (256, 256, 3),
                "names": ["height", "width", "channel"],
            },
            "state": {
                "dtype": "float32",
                "shape": (8,),  # 7个关节角 + 1个夹爪宽度
                "names": ["state"],
            },
            "actions": {
                "dtype": "float32",
                "shape": (7,),  # 3个位置 + 4个四元数
                "names": ["actions"],
            },
        },
        image_writer_threads=10,
        image_writer_processes=5,
    )

    # 获取数据目录中的所有HDF5文件
    data_path = Path(data_dir)
    h5_files = list(data_path.glob("*.h5"))
    
    if not h5_files:
        print(f"在{data_dir}中未找到任何HDF5文件!")
        return
    
    print(f"找到{len(h5_files)}个HDF5文件，开始转换...")
    
    # 遍历每个H5文件，每个文件作为一个episode
    for i, h5_file in enumerate(h5_files):
        print(f"处理文件 {i+1}/{len(h5_files)}: {h5_file.name}")
        
        with h5py.File(h5_file, 'r') as f:
            # 检查文件中的数据集是否符合预期
            required_keys = [
                "observation.images.image", 
                "observation.images.wrist_image", 
                "observation.state", 
                "action"
            ]
            
            if not all(key in f for key in required_keys):
                print(f"警告: 文件{h5_file.name}缺少必要的数据集，跳过此文件")
                print(f"  文件中的数据集: {list(f.keys())}")
                print(f"  所需的数据集: {required_keys}")
                continue
            
            num_frames = len(f["timestamps"])
            if num_frames == 0:
                print(f"警告: 文件{h5_file.name}不包含任何帧，跳过此文件")
                continue
            
            print(f"  加载{num_frames}帧数据...")
            
            # 逐帧添加数据
            for j in range(num_frames):
                # 读取各种数据
                global_image = f["observation.images.image"][j]
                wrist_image = f["observation.images.wrist_image"][j]
                state = f["observation.state"][j]
                action = f["action"][j]
                
                # 添加帧
                dataset.add_frame(
                    {
                        "image": global_image,
                        "wrist_image": wrist_image,
                        "state": state,
                        "actions": action,  # 注意这里变成了actions而不是action
                        # "task": [task_description.encode('utf-8')],  # 添加任务描述作为task特征
                    }
                )
            
            # 保存episode，使用默认任务描述或文件名作为任务
            dataset.save_episode(episode_tasks=[task_description])
            print(f"  已保存episode: {task_description}")

    # 整合数据集，跳过计算统计信息
    print("正在整合数据集...")
    dataset.consolidate(run_compute_stats=False)
    print(f"数据集已成功转换并保存到: {output_path}")

    # 可选：上传到Hugging Face Hub
    if push_to_hub:
        print("正在上传数据集到Hugging Face Hub...")
        dataset.push_to_hub(
            tags=["franka", "teleoperation", "custom_data"],
            private=False,
            push_videos=True,
            license="apache-2.0",
        )
        print(f"数据集已上传到: {REPO_NAME}")


if __name__ == "__main__":
    tyro.cli(main)

