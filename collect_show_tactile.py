#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) Pengwei Zhang -- 2025.5.3
# Author: Pengwei Zhang
# License: MIT

import time
import h5py
import numpy as np
import cv2
import threading
import os
from datetime import datetime
import signal
import sys
import pyrealsense2 as rs
from polymetis import RobotInterface, GripperInterface

# === 新增：导入本地 GelSight SDK ===
# 添加本地gelsight路径到系统路径，使用相对路径
script_dir = os.path.dirname(os.path.abspath(__file__))
# 向上一级目录查找gsrobotics文件夹
gelsight_path = os.path.join(os.path.dirname(script_dir), "gsrobotics")
if gelsight_path not in sys.path:
    sys.path.insert(0, gelsight_path)

# 现在导入本地gelsight模块
from gelmini import gsdevice
from gelmini.gs3drecon import Reconstruction3D, Visualize3D

# 替换keyboard库，使用更兼容的pynput库或者readchar库进行键盘输入处理
try:
    import readchar
    USE_READCHAR = True
except ImportError:
    try:
        from pynput import keyboard as kb
        USE_READCHAR = False
    except ImportError:
        print("请安装readchar或pynput库: pip install readchar pynput")
        sys.exit(1)

class DataCollector:
    """机械臂遥操作数据采集器"""
    
    def __init__(self, freq=10, save_dir="./collected_data_right", 
                 max_buffer_size=1000, save_tactile_pointcloud=True, show_tactile=False,
                 save_tactile_video=False, save_depth=False, save_realsense_pointcloud=2):
        """
        初始化数据采集器
        Args:
            freq: 采集频率 (Hz)
            save_dir: 数据保存目录
            max_buffer_size: 数据缓冲区最大帧数，超过此值会触发自动保存
            save_tactile_pointcloud: 是否保存触觉点云数据
            show_tactile: 是否在采集过程中显示触觉图像
            save_tactile_video: 是否将触觉图像保存为视频
            save_depth: 是否保存Realsense深度图数据
            save_realsense_pointcloud: 是否保存Realsense点云数据 (0: 不保存, 1: 保存无颜色点云, 11: 保存XYZRGB带颜色点云)
        """
        self.serial_1 = "136622074722"  # 全局相机序列号
        self.serial_2 = "233622071355"  # 腕部相机序列号
        self.freq = freq
        self.period = 1.0 / freq
        # 将保存目录转换为绝对路径，确保正确保存数据
        self.save_dir = os.path.abspath(save_dir)
        self.is_collecting = False
        self.exit_flag = False
        self.max_buffer_size = max_buffer_size
        self.save_tactile_pointcloud = save_tactile_pointcloud  # 是否保存触觉点云数据
        self.show_tactile = show_tactile  # 是否显示触觉图像
        self.save_tactile_video = save_tactile_video  # 是否保存触觉视频
        self.save_depth = save_depth  # 是否保存Realsense深度图
        self.save_realsense_pointcloud = save_realsense_pointcloud  # 是否保存Realsense点云数据
        # 视频编码器和视频输出对象（分别为两个传感器）
        self.video_writer1 = None  # 第一个传感器的视频写入器
        self.video_writer2 = None  # 第二个传感器的视频写入器
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用MP4格式
        # 记录时间戳，用于命名视频文件
        self.video_timestamp = None
        
        # === 修改：在 data_buffer 中添加 GelSight 数据存储键 ===
        # 基础数据
        self.data_buffer = {
            "observation.images.image": [],
            "observation.images.wrist_image": [],
            "observation.state": [],
            "observation.ee_pose": [],
            "action": [],
            "timestamps": [],
            "observation.tactile.img1": [],
            "observation.tactile.img2": [],
        }
        
        # 根据配置添加额外数据键
        if self.save_depth:
            self.data_buffer["observation.images.depth_image"] = []
            self.data_buffer["observation.images.wrist_depth_image"] = []
            
        if self.save_tactile_pointcloud:
            self.data_buffer["observation.tactile.pc1"] = []
            self.data_buffer["observation.tactile.pc2"] = []
            
        if self.save_realsense_pointcloud == 0:
            # 无颜色点云 (XYZ)
            self.data_buffer["observation.images.pointcloud"] = []
            self.data_buffer["observation.images.wrist_pointcloud"] = []
        elif self.save_realsense_pointcloud == 1:
            # 带颜色点云 (XYZRGB)
            self.data_buffer["observation.images.pointcloud_xyzrgb"] = []
            self.data_buffer["observation.images.wrist_pointcloud_xyzrgb"] = []
        
        self.robot_ip = "172.16.97.210"     # 机械臂和夹爪的IP地址------修改为你的实际IP地址
        self.gripper_ip = "172.16.97.210"   # 机械臂和夹爪的IP地址------修改为你的实际IP地址
        # self.robot_ip = "localhost"
        # self.gripper_ip = "localhost"
        self.robot = RobotInterface(ip_address=self.robot_ip, enforce_version=False)
        # ---------修改默认的roboteinterface后，要加上enforce_version=False参数-------- 
        self.gripper = GripperInterface(ip_address=self.gripper_ip)
        
        # === 新增：初始化两个 GelSight 传感器 ===
        try:
            print("初始化 GelSight 传感器1")
            # 使用直接的摄像头ID而不是名称，避免重复连接到同一设备
            self.gel1 = gsdevice.Camera()

            self.gel1.connect()
        except Exception as e:
            print(f"错误: 无法连接第一个 GelSight 传感器: {e}")
            sys.exit(1)
        
        try:
            print("初始化 GelSight 传感器2")
            self.gel2 = gsdevice.Camera()

            self.gel2.connect()
        except Exception as e:
            print(f"错误: 无法连接第二个 GelSight 传感器: {e}")
            print("两个传感器都必须连接才能继续，程序将退出")
            # 关闭已连接的传感器，释放摄像头资源
            if hasattr(self.gel1, 'cam') and self.gel1.cam is not None:
                self.gel1.cam.release()
            sys.exit(1)
        
        # === 新增：初始化点云重建模型 ===
        if self.save_tactile_pointcloud:
            print("初始化点云重建模型")
            self.mmpp = 0.0634
            # 使用当前脚本所在目录查找模型文件
            script_dir = os.path.dirname(os.path.abspath(__file__))
            net_path = os.path.join(script_dir, "nnmini.pt")
            if not os.path.isfile(net_path):
                raise FileNotFoundError(f"找不到模型 nnmini.pt，请放在：{script_dir}")
        
            # 初始化第一个传感器的重建器和可视化器
            self.reconstructor1 = Reconstruction3D(self.gel1)
            self.reconstructor1.load_nn(net_path, "cpu")
            print("初始化第一个点云可视化器")
            self.visualizer1 = Visualize3D(self.gel1.imgh, self.gel1.imgw, '', self.mmpp)
            
            # 为第二个传感器初始化重建器和可视化器
            print("初始化第二个点云重建模型")
            self.reconstructor2 = Reconstruction3D(self.gel2)
            self.reconstructor2.load_nn(net_path, "cpu")
            print("初始化第二个点云可视化器")
            self.visualizer2 = Visualize3D(self.gel2.imgh, self.gel2.imgw, '', self.mmpp)
        
        # 确保存储目录存在
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 设置信号处理，确保优雅退出
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 初始化全局相机
        print("初始化全局相机")
        self.global_cam_pipeline = rs.pipeline()
        global_config = rs.config()
        # 使用设备序列号区分全局相机
        global_config.enable_device(self.serial_1)  # 请替换为实际全局相机的序列号
        global_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            # 启用深度流
            global_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.global_cam_pipeline.start(global_config)
        # 创建对齐对象，用于将深度帧对齐到彩色帧
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            self.global_align = rs.align(rs.stream.color)
        
        # 初始化腕部相机
        print("初始化腕部相机")
        self.wrist_cam_pipeline = rs.pipeline()
        wrist_config = rs.config()
        # 使用设备序列号区分腕部相机
        wrist_config.enable_device(self.serial_2)  # 请替换为实际腕部相机的序列号
        wrist_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            # 启用深度流
            wrist_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.wrist_cam_pipeline.start(wrist_config)
        # 创建对齐对象，用于将深度帧对齐到彩色帧
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            self.wrist_align = rs.align(rs.stream.color)
        
        for _ in range(5):
            self.global_cam_pipeline.wait_for_frames()
            self.wrist_cam_pipeline.wait_for_frames()
        
    def _signal_handler(self, sig, frame):
        """处理中断信号，确保数据完整性"""
        print("\n正在安全停止数据采集...")
        self.stop_collecting()
        
    def _get_global_camera_frames(self):
        """
        一次性获取彩色 + 深度 + （可选）点云。
        返回：
          color_image: (256,256,3) RGB ndarray
          depth_image: (256,256) float32 ndarray
          pointcloud:    Open3D 格式或 Nx3 ndarray，若 self.save_pointcloud=False 则 None
        """
        # 一次性等待（可调超时时间，单位 ms）
        frames = self.global_cam_pipeline.wait_for_frames(timeout_ms=5000)

        # 对齐深度到彩色
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            frames = self.global_align.process(frames)

        # 提取彩色帧
        color_frame = frames.get_color_frame()
        color_image_original = np.asanyarray(color_frame.get_data())
        color_image = cv2.resize(color_image_original, (320,240))
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # 提取深度帧
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data(), dtype=np.float32)
            depth_image = cv2.resize(depth_image, (320,240), interpolation=cv2.INTER_NEAREST)
        else:
            depth_image = None

        # 生成点云
        if self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            pc = rs.pointcloud()
            if self.save_realsense_pointcloud == 1:
                # 设置纹理映射
                pc.map_to(color_frame)
            # 将深度帧映射到点云
            points = pc.calculate(depth_frame)
            # 获取每个像素的三维坐标 (单位米)
            vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                    # 获取纹理坐标
            if self.save_realsense_pointcloud == 1:
                textures = np.asanyarray(points.get_texture_coordinates())
                textures = textures.view(np.float32).reshape(-1, 2)  # 转换为N×2的numpy数组
                
                # 计算每个点的颜色
                h, w = color_image_original.shape[:2]
                colors = np.zeros((vtx.shape[0], 3), dtype=np.uint8)
                for i, (u, v) in enumerate(textures):
                    if 0 <= u <= 1 and 0 <= v <= 1:
                        x = min(int(u * w), w - 1)
                        y = min(int(v * h), h - 1)
                        colors[i] = color_image_original[y, x]
                
                # 创建XYZRGB格式的点云 (N,6)
                xyzrgb = np.zeros((vtx.shape[0], 6), dtype=np.float32)
                xyzrgb[:, 0:3] = vtx  # XYZ
                xyzrgb[:, 3:6] = colors.astype(np.float32) / 255.0  # RGB (归一化到0-1)
                pointcloud = xyzrgb  # shape: [N, 6]
            elif self.save_realsense_pointcloud == 0:
                # ---------------如果需要，可以下采样或过滤，这里直接返回所有点----------
                pointcloud = vtx
        else:
            pointcloud = None

        return color_image, depth_image, pointcloud
    
    # def _get_global_camera_depth(self):
    #     """获取全局相机校正后的深度图
        
    #     Returns:
    #         numpy.ndarray: 形状为(256, 256)的深度图，单位为毫米
    #     """
    #     if not self.save_depth:
    #         return np.zeros((256, 256), dtype=np.float32)
            
    #     frames = self.global_cam_pipeline.wait_for_frames()
        
    #     # 对齐深度帧和彩色帧
    #     aligned_frames = self.global_align.process(frames)
        
    #     # 获取对齐后的深度帧
    #     depth_frame = aligned_frames.get_depth_frame()
        
    #     if not depth_frame:
    #         return np.zeros((256, 256), dtype=np.float32)
        
    #     # 转换为numpy数组
    #     depth_image = np.asanyarray(depth_frame.get_data())
        
    #     # 将图像缩放到256x256
    #     depth_image = cv2.resize(depth_image, (256, 256), interpolation=cv2.INTER_NEAREST)
        
    #     # 转换为float32类型，单位为毫米
    #     depth_image = depth_image.astype(np.float32)
        
    #     return depth_image
    
    def _get_wrist_camera_frames(self):
        """
        一次性获取彩色 + 深度 + （可选）点云。
        返回：
          color_image: (256,256,3) RGB ndarray
          depth_image: (256,256) float32 ndarray
          pointcloud:    Open3D 格式或 Nx3 ndarray，若 self.save_pointcloud=False 则 None
        """
        # 一次性等待（可调超时时间，单位 ms）
        frames = self.wrist_cam_pipeline.wait_for_frames()

        # 对齐深度到彩色
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            frames = self.wrist_align.process(frames)

        # 提取彩色帧
        color_frame = frames.get_color_frame()
        color_image_original = np.asanyarray(color_frame.get_data())
        color_image = cv2.resize(color_image_original, (320,240))
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # 提取深度帧
        if self.save_depth or self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data(), dtype=np.float32)
            depth_image = cv2.resize(depth_image, (320,240), interpolation=cv2.INTER_NEAREST)
        else:
            depth_image = None

        # 生成点云
        if self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            pc = rs.pointcloud()
            if self.save_realsense_pointcloud == 1:
                # 设置纹理映射
                pc.map_to(color_frame)
            # 将深度帧映射到点云
            points = pc.calculate(depth_frame)
            # 获取每个像素的三维坐标 (单位米)
            vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                    # 获取纹理坐标
            if self.save_realsense_pointcloud == 1:
                textures = np.asanyarray(points.get_texture_coordinates())
                textures = textures.view(np.float32).reshape(-1, 2)  # 转换为N×2的numpy数组
                
                # 计算每个点的颜色
                h, w = color_image_original.shape[:2]
                colors = np.zeros((vtx.shape[0], 3), dtype=np.uint8)
                for i, (u, v) in enumerate(textures):
                    if 0 <= u <= 1 and 0 <= v <= 1:
                        x = min(int(u * w), w - 1)
                        y = min(int(v * h), h - 1)
                        colors[i] = color_image_original[y, x]
                
                # 创建XYZRGB格式的点云 (N,6)
                xyzrgb = np.zeros((vtx.shape[0], 6), dtype=np.float32)
                xyzrgb[:, 0:3] = vtx  # XYZ
                xyzrgb[:, 3:6] = colors.astype(np.float32) / 255.0  # RGB (归一化到0-1)
                pointcloud = xyzrgb  # shape: [N, 6]
            elif self.save_realsense_pointcloud == 0:
                # ---------------如果需要，可以下采样或过滤，这里直接返回所有点----------
                pointcloud = vtx
        else:
            pointcloud = None

        return color_image, depth_image, pointcloud
    
    # def _get_wrist_camera_depth(self):
    #     """获取腕部相机校正后的深度图
        
    #     Returns:
    #         numpy.ndarray: 形状为(256, 256)的深度图，单位为毫米
    #     """
    #     if not self.save_depth:
    #         return np.zeros((256, 256), dtype=np.float32)
            
    #     frames = self.wrist_cam_pipeline.wait_for_frames()
        
    #     # 对齐深度帧和彩色帧
    #     aligned_frames = self.wrist_align.process(frames)
        
    #     # 获取对齐后的深度帧
    #     depth_frame = aligned_frames.get_depth_frame()
        
    #     if not depth_frame:
    #         return np.zeros((256, 256), dtype=np.float32)
        
    #     # 转换为numpy数组
    #     depth_image = np.asanyarray(depth_frame.get_data())
        
    #     # 将图像缩放到256x256
    #     depth_image = cv2.resize(depth_image, (256, 256), interpolation=cv2.INTER_NEAREST)
        
    #     # 转换为float32类型，单位为毫米
    #     depth_image = depth_image.astype(np.float32)
        
    #     return depth_image
    
    def _get_robot_state(self):
        """获取机械臂的关节角和夹爪宽度
        
        Returns:
            numpy.ndarray: 形状为(8,)的浮点数数组，包含7个关节角和1个夹爪宽度
        """
        # 获取机械臂关节角度
        joint_state = self.robot.get_joint_positions()
        
        # 获取夹爪宽度
        gripper_width = self.gripper.get_state().width
        # 将关节角和夹爪宽度拼接
        robot_state = np.concatenate([joint_state, np.array([gripper_width])]).astype(np.float32)
        
        return robot_state
    
    def _get_robot_ee_pose(self):
        """获取机械臂末端位姿(3位置+4四元数)
        
        Returns:
            numpy.ndarray: 形状为(7,)的浮点数数组，包含3个位置和4个四元数
        """
        pos, quat = self.robot.get_ee_pose()
        # 确保四元数归一化
        # quat = np.random.uniform(-1, 1, (4,))
        # quat = quat / np.linalg.norm(quat)
        
        return np.concatenate([pos, quat]).astype(np.float32)
    
    def _get_robot_desired_action(self):
        """获取机械臂期望的关节角位置和夹爪宽度
        
        Returns:
            numpy.ndarray: 形状为(8,)的浮点数数组，包含7个期望关节角和1个夹爪目标宽度
        """
        # 获取期望的关节角度
        desired_joint_positions = self.robot.get_desired_joint_positions()
        
        # 如果没有设置期望关节位置，则使用当前关节位置
        if desired_joint_positions is None:
            desired_joint_positions = self.robot.get_joint_positions()
        
        # 获取夹爪的目标宽度
        gripper_params = self.gripper.get_desired_gripper_goto_params()
        
        # 如果没有设置夹爪参数，则使用当前夹爪宽度
        if gripper_params is None:
            gripper_width = self.gripper.get_state().width
        else:
            gripper_width = gripper_params["width"]
        
        # 将期望关节角和夹爪目标宽度拼接
        desired_state = np.concatenate([desired_joint_positions, np.array([gripper_width])]).astype(np.float32)
        
        return desired_state
    
    # === 新增：获取 GelSight 点云 ===
    def _get_gelsight_pointcloud(self, sensor, sensor_id):
        """
        用神经网络+Poisson 求解深度，生成 N×3 点云
        
        Args:
            sensor: GelSight传感器实例
            sensor_id: 传感器ID（1或2）
        
        Returns:
            numpy.ndarray: 点云数据，形状为(N,3)
        """
        # 1) 先拿 2D 图
        img = sensor.get_image()
        if img is None:
            print(f"WARNING: 无法从 GelSight {sensor_id} 读取图像，返回空点云")
            return np.empty((0, 3))

        # 2) 选用对应的 reconstructor 和 visualizer
        if sensor_id == 1:
            rec = self.reconstructor1
            vis = self.visualizer1
        else:
            rec = self.reconstructor2
            vis = self.visualizer2

        # 3) 得到深度图（mm）
        depth_map = rec.get_depthmap(img, False)

        # 4 使用 Visualize3D 更新和保存点云
        vis.update(depth_map)

        return vis.points.copy()
    
    # def _get_global_camera_pointcloud(self):
    #     """获取全局相机点云数据
        
    #     Returns:
    #         numpy.ndarray: 点云数据，形状为(N,3)的numpy数组
    #     """
    #     if self.save_realsense_pointcloud == 2:
    #         return np.zeros((0, 3), dtype=np.float32)
            
    #     # 获取帧
    #     frames = self.global_cam_pipeline.wait_for_frames()
        
    #     # 对齐深度帧和彩色帧
    #     aligned_frames = self.global_align.process(frames)
        
    #     # 获取对齐后的深度帧
    #     depth_frame = aligned_frames.get_depth_frame()
        
    #     if not depth_frame:
    #         return np.zeros((0, 3), dtype=np.float32)
        
    #     # 创建点云
    #     pc = rs.pointcloud()
        
    #     # 从深度帧计算点云
    #     points = pc.calculate(depth_frame)
        
    #     # 获取顶点数据
    #     vertices = np.asanyarray(points.get_vertices())
    #     vertices = vertices.view(np.float32).reshape(-1, 3)  # 转换为N×3的numpy数组
        
    #     return vertices
    
    # def _get_wrist_camera_pointcloud(self):
    #     """获取腕部相机点云数据
        
    #     Returns:
    #         numpy.ndarray: 点云数据，形状为(N,3)的numpy数组
    #     """
    #     if self.save_realsense_pointcloud == 2:
    #         return np.zeros((0, 3), dtype=np.float32)
            
    #     # 获取帧
    #     frames = self.wrist_cam_pipeline.wait_for_frames()
        
    #     # 对齐深度帧和彩色帧
    #     aligned_frames = self.wrist_align.process(frames)
        
    #     # 获取对齐后的深度帧
    #     depth_frame = aligned_frames.get_depth_frame()
        
    #     if not depth_frame:
    #         return np.zeros((0, 3), dtype=np.float32)
        
    #     # 创建点云
    #     pc = rs.pointcloud()
        
    #     # 从深度帧计算点云
    #     points = pc.calculate(depth_frame)
        
    #     # 获取顶点数据
    #     vertices = np.asanyarray(points.get_vertices())
    #     vertices = vertices.view(np.float32).reshape(-1, 3)  # 转换为N×3的numpy数组
        
    #     return vertices
    
    # def _get_global_camera_pointcloud_xyzrgb(self):
    #     """获取全局相机带颜色的点云数据 (XYZRGB)
        
    #     Returns:
    #         numpy.ndarray: 带颜色点云数据，形状为(N,6)的numpy数组，每行为[x,y,z,r,g,b]
    #     """
    #     if self.save_realsense_pointcloud != 1:
    #         return np.zeros((0, 6), dtype=np.float32)
            
    #     # 获取帧
    #     frames = self.global_cam_pipeline.wait_for_frames()
        
    #     # 对齐深度帧和彩色帧
    #     aligned_frames = self.global_align.process(frames)
        
    #     # 获取对齐后的深度帧和彩色帧
    #     depth_frame = aligned_frames.get_depth_frame()
    #     color_frame = aligned_frames.get_color_frame()
        
    #     if not depth_frame or not color_frame:
    #         return np.zeros((0, 6), dtype=np.float32)
        
    #     # 创建点云
    #     pc = rs.pointcloud()
        
    #     # 设置纹理映射
    #     pc.map_to(color_frame)
        
    #     # 从深度帧计算点云
    #     points = pc.calculate(depth_frame)
        
    #     # 获取顶点数据
    #     vertices = np.asanyarray(points.get_vertices())
    #     vertices = vertices.view(np.float32).reshape(-1, 3)  # 转换为N×3的numpy数组
        
    #     # 获取纹理坐标
    #     textures = np.asanyarray(points.get_texture_coordinates())
    #     textures = textures.view(np.float32).reshape(-1, 2)  # 转换为N×2的numpy数组
        
    #     # 获取彩色图像
    #     color_image = np.asanyarray(color_frame.get_data())
        
    #     # 计算每个点的颜色
    #     h, w = color_image.shape[:2]
    #     colors = np.zeros((vertices.shape[0], 3), dtype=np.uint8)
    #     for i, (u, v) in enumerate(textures):
    #         if 0 <= u <= 1 and 0 <= v <= 1:
    #             x = min(int(u * w), w - 1)
    #             y = min(int(v * h), h - 1)
    #             colors[i] = color_image[y, x]
        
    #     # 创建XYZRGB格式的点云 (N,6)
    #     xyzrgb = np.zeros((vertices.shape[0], 6), dtype=np.float32)
    #     xyzrgb[:, 0:3] = vertices  # XYZ
    #     xyzrgb[:, 3:6] = colors.astype(np.float32) / 255.0  # RGB (归一化到0-1)
        
    #     return xyzrgb
    
    # def _get_wrist_camera_pointcloud_xyzrgb(self):
    #     """获取腕部相机带颜色的点云数据 (XYZRGB)
        
    #     Returns:
    #         numpy.ndarray: 带颜色点云数据，形状为(N,6)的numpy数组，每行为[x,y,z,r,g,b]
    #     """
    #     if self.save_realsense_pointcloud != 1:
    #         return np.zeros((0, 6), dtype=np.float32)
            
    #     # 获取帧
    #     frames = self.wrist_cam_pipeline.wait_for_frames()
        
    #     # 对齐深度帧和彩色帧
    #     aligned_frames = self.wrist_align.process(frames)
        
    #     # 获取对齐后的深度帧和彩色帧
    #     depth_frame = aligned_frames.get_depth_frame()
    #     color_frame = aligned_frames.get_color_frame()
        
    #     if not depth_frame or not color_frame:
    #         return np.zeros((0, 6), dtype=np.float32)
        
    #     # 创建点云
    #     pc = rs.pointcloud()
        
    #     # 设置纹理映射
    #     pc.map_to(color_frame)
        
    #     # 从深度帧计算点云
    #     points = pc.calculate(depth_frame)
        
    #     # 获取顶点数据
    #     vertices = np.asanyarray(points.get_vertices())
    #     vertices = vertices.view(np.float32).reshape(-1, 3)  # 转换为N×3的numpy数组
        
    #     # 获取纹理坐标
    #     textures = np.asanyarray(points.get_texture_coordinates())
    #     textures = textures.view(np.float32).reshape(-1, 2)  # 转换为N×2的numpy数组
        
    #     # 获取彩色图像
    #     color_image = np.asanyarray(color_frame.get_data())
        
    #     # 计算每个点的颜色
    #     h, w = color_image.shape[:2]
    #     colors = np.zeros((vertices.shape[0], 3), dtype=np.uint8)
    #     for i, (u, v) in enumerate(textures):
    #         if 0 <= u <= 1 and 0 <= v <= 1:
    #             x = min(int(u * w), w - 1)
    #             y = min(int(v * h), h - 1)
    #             colors[i] = color_image[y, x]
        
    #     # 创建XYZRGB格式的点云 (N,6)
    #     xyzrgb = np.zeros((vertices.shape[0], 6), dtype=np.float32)
    #     xyzrgb[:, 0:3] = vertices  # XYZ
    #     xyzrgb[:, 3:6] = colors.astype(np.float32) / 255.0  # RGB (归一化到0-1)
        
    #     return xyzrgb
    
    def _collect_data_point(self):
        """采集一帧数据"""
        # 检查缓冲区大小，如果超过最大限制，自动保存数据
        if len(self.data_buffer["timestamps"]) >= self.max_buffer_size:
            print(f"\n缓冲区已达到最大限制({self.max_buffer_size}帧)，正在自动保存数据...")
            self._save_data()
            print("数据已保存，继续采集...")
            
        global_img, global_depth, global_pc = self._get_global_camera_frames()
        wrist_img, wrist_depth, wrist_pc = self._get_wrist_camera_frames()
        robot_state = self._get_robot_state()
        robot_ee_pose = self._get_robot_ee_pose()
        robot_desired_action = self._get_robot_desired_action()
        timestamp = time.time()
        
        # 获取Realsense深度图（如果启用）
        if self.save_depth:
            self.data_buffer["observation.images.depth_image"].append(global_depth)
            self.data_buffer["observation.images.wrist_depth_image"].append(wrist_depth)
        
        # 获取Realsense点云（如果启用）
        if self.save_realsense_pointcloud == 0 or self.save_realsense_pointcloud == 1:
            # 根据选项决定采集普通点云还是带颜色的点云
            if self.save_realsense_pointcloud == 0:
                self.data_buffer["observation.images.pointcloud"].append(global_pc)
                self.data_buffer["observation.images.wrist_pointcloud"].append(wrist_pc)
            elif self.save_realsense_pointcloud == 1:
                # 采集带颜色XYZRGB点云
                self.data_buffer["observation.images.pointcloud_xyzrgb"].append(global_pc)
                self.data_buffer["observation.images.wrist_pointcloud_xyzrgb"].append(wrist_pc)
        
        # === 采集触觉图像和点云 ===
        # 采集两个传感器的数据
        tac_img1 = self.gel1.get_image()
        tac_img2 = self.gel2.get_image()
        
        # 处理触觉图像（显示和保存视频）
        if tac_img1 is not None and tac_img2 is not None:
            # 获取当前帧计数
            frame_count = len(self.data_buffer["timestamps"])
            
            # 准备用于显示的图像
            if self.show_tactile:
                # 制作显示用的拷贝，添加标题
                display_img1 = tac_img1.copy()
                display_img2 = tac_img2.copy()
                
                # 添加标题
                cv2.putText(display_img1, f"GelSight 1", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img2, f"GelSight 2", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 确保两个图像具有相同的尺寸
                if display_img1.shape != display_img2.shape:
                    # 调整第二个图像到与第一个相同的尺寸
                    display_img2 = cv2.resize(display_img2, (display_img1.shape[1], display_img1.shape[0]))
                
                # 水平拼接两个图像
                combined_img = np.hstack((display_img1, display_img2))
                
                # 在顶部添加帧计数信息
                info_bar = np.zeros((40, combined_img.shape[1], 3), dtype=np.uint8)
                cv2.putText(info_bar, f"Frame: {frame_count}", 
                          (combined_img.shape[1]//2 - 60, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                # 垂直拼接信息栏和图像
                final_display = np.vstack((info_bar, combined_img))
                
                # 显示在一个窗口中
                cv2.imshow("GelSight Tactile Sensors", final_display)
                cv2.waitKey(1)
            
            # 处理视频保存（两个独立视频）
            if self.save_tactile_video:
                # 如果还没有保存时间戳，创建一个新的时间戳
                if self.video_timestamp is None:
                    self.video_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                
                # 为第一个传感器初始化视频写入器
                if self.video_writer1 is None and tac_img1 is not None:
                    video_height, video_width = tac_img1.shape[:2]
                    video_path1 = os.path.join(self.save_dir, f"tactile_video1_{self.video_timestamp}.mp4")
                    self.video_writer1 = cv2.VideoWriter(
                        video_path1, self.fourcc, self.freq, (video_width, video_height))
                    print(f"开始录制传感器1的触觉视频到 {os.path.relpath(video_path1, os.getcwd())}")
                
                # 为第二个传感器初始化视频写入器
                if self.video_writer2 is None and tac_img2 is not None:
                    video_height, video_width = tac_img2.shape[:2]
                    video_path2 = os.path.join(self.save_dir, f"tactile_video2_{self.video_timestamp}.mp4")
                    self.video_writer2 = cv2.VideoWriter(
                        video_path2, self.fourcc, self.freq, (video_width, video_height))
                    print(f"开始录制传感器2的触觉视频到 {os.path.relpath(video_path2, os.getcwd())}")
                
                # 将图像帧写入各自的视频文件
                if self.video_writer1 is not None and tac_img1 is not None:
                    # 确保图像是彩色BGR格式
                    if len(tac_img1.shape) == 2:  # 灰度图像
                        tac_img1_bgr = cv2.cvtColor(tac_img1, cv2.COLOR_GRAY2BGR)
                    else:  # 已经是彩色图像
                        tac_img1_bgr = tac_img1.copy()
                    self.video_writer1.write(tac_img1_bgr)
                
                if self.video_writer2 is not None and tac_img2 is not None:
                    # 确保图像是彩色BGR格式
                    if len(tac_img2.shape) == 2:  # 灰度图像
                        tac_img2_bgr = cv2.cvtColor(tac_img2, cv2.COLOR_GRAY2BGR)
                    else:  # 已经是彩色图像
                        tac_img2_bgr = tac_img2.copy()
                    self.video_writer2.write(tac_img2_bgr)
        
        # 根据配置决定是否采集点云
        if self.save_tactile_pointcloud:
            # 采集第一个传感器的点云
            tac_pc1 = self._get_gelsight_pointcloud(self.gel1, 1)
            self.data_buffer["observation.tactile.pc1"].append(tac_pc1)
            
            # 采集第二个传感器的点云
            tac_pc2 = self._get_gelsight_pointcloud(self.gel2, 2)
            self.data_buffer["observation.tactile.pc2"].append(tac_pc2)
        
        # 添加到缓冲区
        self.data_buffer["observation.images.image"].append(global_img)
        self.data_buffer["observation.images.wrist_image"].append(wrist_img)
        self.data_buffer["observation.state"].append(robot_state)
        self.data_buffer["observation.ee_pose"].append(robot_ee_pose)
        self.data_buffer["action"].append(robot_desired_action)
        self.data_buffer["timestamps"].append(timestamp)
        # 添加触觉数据到缓冲区
        self.data_buffer["observation.tactile.img1"].append(tac_img1)
        self.data_buffer["observation.tactile.img2"].append(tac_img2)
        
        return timestamp
    
    def _collection_loop(self):
        """数据采集主循环"""
        self.is_collecting = True
        print("开始数据采集，按回车键结束当前采集...")
        
        count = 0
        start_time = time.time()
        last_time = start_time
        
        while self.is_collecting and not self.exit_flag:
            loop_start = time.time()
            
            # 采集数据
            timestamp = self._collect_data_point()
            count += 1
            
            # 计算并打印当前采集频率
            if count % 10 == 0:
                current_time = time.time()
                elapsed = current_time - last_time
                freq = 10 / elapsed if elapsed > 0 else 0
                print(f"\r当前采集频率: {freq:.2f} Hz, 已采集帧数: {count}", end="")
                last_time = current_time
                
            # 精确控制采集频率
            elapsed = time.time() - loop_start
            sleep_time = max(0, self.period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        total_time = time.time() - start_time
        avg_freq = count / total_time if total_time > 0 else 0
        print(f"\n采集结束，共采集 {count} 帧数据，平均采集频率: {avg_freq:.2f} Hz")
        
        # 采集结束后保存数据
        if count > 0:
            self._save_data()
        
        # 关闭显示窗口（如果有）
        if self.show_tactile:
            cv2.destroyAllWindows()
    
    def _save_data(self):
        """将采集的数据保存为HDF5格式"""
        if not self.data_buffer["timestamps"]:
            print("没有数据需要保存")
            return
            
        # 生成一个包含日期时间的唯一文件名
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        count = len(self.data_buffer["timestamps"])
        filename = os.path.join(self.save_dir, f"pick_data_{timestamp_str}_{count}frames.h5")
        
        print(f"正在保存数据到 {filename}...")
        
        # 保存HDF5格式的数据
        with h5py.File(filename, 'w') as f:
            # 保存时间戳
            f.create_dataset("timestamps", 
                            data=np.array(self.data_buffer["timestamps"]))
            
            # 保存全局相机图像
            f.create_dataset("observation.images.image", 
                            data=np.array(self.data_buffer["observation.images.image"]),
                            dtype='uint8',
                            compression="gzip", 
                            compression_opts=4)
            
            # 保存腕部相机图像
            f.create_dataset("observation.images.wrist_image", 
                            data=np.array(self.data_buffer["observation.images.wrist_image"]),
                            dtype='uint8',
                            compression="gzip", 
                            compression_opts=4)
            
            # === 保存深度图数据（如果有） ===
            if self.save_depth:
                f.create_dataset("observation.images.depth_image", 
                               data=np.array(self.data_buffer["observation.images.depth_image"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
                
                f.create_dataset("observation.images.wrist_depth_image", 
                               data=np.array(self.data_buffer["observation.images.wrist_depth_image"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
            
            # 保存机械臂状态
            f.create_dataset("observation.state", 
                            data=np.array(self.data_buffer["observation.state"]),
                            dtype='float32')
            
            # 保存机械臂动作
            f.create_dataset("observation.ee_pose", 
                            data=np.array(self.data_buffer["observation.ee_pose"]),
                            dtype='float32')
            
            # 保存机械臂期望状态
            f.create_dataset("action", 
                            data=np.array(self.data_buffer["action"]),
                            dtype='float32')
            
            # === 保存触觉图像 ===
            f.create_dataset("observation.tactile.img1", 
                           data=np.array(self.data_buffer["observation.tactile.img1"]),
                           dtype='uint8',
                           compression="gzip", 
                           compression_opts=4)
            
            f.create_dataset("observation.tactile.img2", 
                           data=np.array(self.data_buffer["observation.tactile.img2"]),
                           dtype='uint8',
                           compression="gzip", 
                           compression_opts=4)
            
            # === 保存触觉点云数据（如果有） ===
            if self.save_tactile_pointcloud:
                f.create_dataset("observation.tactile.pc1", 
                               data=np.array(self.data_buffer["observation.tactile.pc1"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
                
                f.create_dataset("observation.tactile.pc2", 
                               data=np.array(self.data_buffer["observation.tactile.pc2"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
            
            # === 保存Realsense点云数据（如果有） ===
            if self.save_realsense_pointcloud == 0:
                # 保存普通点云数据 (XYZ)
                f.create_dataset("observation.images.pointcloud", 
                               data=np.array(self.data_buffer["observation.images.pointcloud"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
                
                f.create_dataset("observation.images.wrist_pointcloud", 
                               data=np.array(self.data_buffer["observation.images.wrist_pointcloud"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
            
            if self.save_realsense_pointcloud == 1:
                # 保存带颜色的点云数据 (XYZRGB)
                f.create_dataset("observation.images.pointcloud_xyzrgb", 
                               data=np.array(self.data_buffer["observation.images.pointcloud_xyzrgb"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
                
                f.create_dataset("observation.images.wrist_pointcloud_xyzrgb", 
                               data=np.array(self.data_buffer["observation.images.wrist_pointcloud_xyzrgb"]),
                               dtype='float32',
                               compression="gzip", 
                               compression_opts=4)
            
        # 使用相对路径显示结果
        rel_filename = os.path.relpath(filename, os.getcwd())
        print(f"数据成功保存到 {rel_filename}")
        
        # 如果在录制视频，关闭视频写入器并创建新的视频
        if self.save_tactile_video:
            if self.video_writer1 is not None:
                self.video_writer1.release()
                self.video_writer1 = None
            
            if self.video_writer2 is not None:
                self.video_writer2.release()
                self.video_writer2 = None
                
            print("触觉视频已保存")
            # 重置时间戳，下次会创建新的视频文件
            self.video_timestamp = None
        
        # 清空缓冲区
        for key in self.data_buffer:
            self.data_buffer[key] = []
    
    def start_collecting(self):
        """开始数据采集"""
        if self.is_collecting:
            print("数据采集已经在进行中")
            return
            
        # 创建并启动采集线程
        self.collection_thread = threading.Thread(target=self._collection_loop)
        self.collection_thread.daemon = True
        self.collection_thread.start()
    
    def stop_collecting(self):
        """停止数据采集"""
        if not self.is_collecting:
            print("没有正在进行的数据采集")
            return
            
        self.is_collecting = False
        if hasattr(self, 'collection_thread') and self.collection_thread.is_alive():
            self.collection_thread.join()
        
        # 关闭视频写入器（如果有）
        if self.save_tactile_video:
            if hasattr(self, 'video_writer1') and self.video_writer1 is not None:
                self.video_writer1.release()
                self.video_writer1 = None
            
            if hasattr(self, 'video_writer2') and self.video_writer2 is not None:
                self.video_writer2.release()
                self.video_writer2 = None
    
    def exit(self):
        """安全退出程序"""
        self.exit_flag = True
        self.stop_collecting()
        
        # 关闭相机流
        if hasattr(self, 'global_cam_pipeline'):
            self.global_cam_pipeline.stop()
        if hasattr(self, 'wrist_cam_pipeline'):
            self.wrist_cam_pipeline.stop()
        
        # === 释放 GelSight 相机资源 ===
        if hasattr(self, 'gel1') and hasattr(self.gel1, 'cam') and self.gel1.cam is not None:
            self.gel1.cam.release()
        if hasattr(self, 'gel2') and hasattr(self.gel2, 'cam') and self.gel2.cam is not None:
            self.gel2.cam.release()
        
        # 关闭视频写入器（如果有）
        if hasattr(self, 'video_writer1') and self.video_writer1 is not None:
            self.video_writer1.release()
        if hasattr(self, 'video_writer2') and self.video_writer2 is not None:
            self.video_writer2.release()
            
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()
    
    def __del__(self):
        """析构函数，确保资源被释放"""
        # 关闭相机流
        if hasattr(self, 'global_cam_pipeline'):
            self.global_cam_pipeline.stop()
        if hasattr(self, 'wrist_cam_pipeline'):
            self.wrist_cam_pipeline.stop()
        # === 释放 GelSight 相机资源 ===
        if hasattr(self, 'gel1') and hasattr(self.gel1, 'cam') and self.gel1.cam is not None:
            self.gel1.cam.release()
        if hasattr(self, 'gel2') and hasattr(self.gel2, 'cam') and self.gel2.cam is not None:
            self.gel2.cam.release()

def wait_for_space():
    """等待用户按下空格键，使用简化的逻辑"""
    print("请按空格键开始采集...")
    
    if USE_READCHAR:
        while True:
            key = readchar.readkey()
            if key == ' ':  # 空格键
                return
    else:
        # 使用更简单的事件标志
        space_pressed = threading.Event()
        
        def on_press(key):
            if hasattr(key, 'char') and key.char == ' ':  # 空格键
                space_pressed.set()
                return False  # 停止监听
            return True
        
        with kb.Listener(on_press=on_press) as listener:
            # 等待空格键被按下或超时
            space_pressed.wait(timeout=600)  # 10分钟超时
            if not space_pressed.is_set():
                print("等待超时，默认开始采集")

def wait_for_enter_to_stop():
    """等待用户按下回车键来停止采集，使用简化的逻辑"""
    print("\n数据采集中，按回车键结束该条数据采集...")
    
    if USE_READCHAR:
        while True:
            key = readchar.readkey()
            if key == readchar.key.ENTER:
                return
    else:
        # 使用更简单的事件标志
        enter_pressed = threading.Event()
        
        def on_press(key):
            if key == kb.Key.enter:
                enter_pressed.set()
                return False  # 停止监听
            return True
        
        with kb.Listener(on_press=on_press) as listener:
            # 等待回车键被按下或超时
            enter_pressed.wait(timeout=3600)  # 1小时超时
            if not enter_pressed.is_set():
                print("等待超时，默认结束采集")

def get_user_choice(prompt, options):
    """获取用户选择
    
    Args:
        prompt: 提示信息
        options: 选项字典，键为选项编号，值为选项描述
        
    Returns:
        用户选择的选项编号
    """
    while True:
        print(prompt)
        for key, value in sorted(options.items()):
            print(f"{key}. {value}")
        
        if USE_READCHAR:
            choice = readchar.readchar()
        else:
            choice = input("请输入选项编号: ").strip()
            
        if choice in options:
            return choice
        else:
            print("无效的选择，请重试!")

def main():
    """主函数"""
    # 初始化数据采集器的参数
    freq = 15
    max_buffer_size = 750
    
    # 设置默认保存目录
    save_dir = "/home/ubuntu/workspace/data/collected_data"
    
    # 获取用户对点云保存的选择
    save_pc_choice = get_user_choice(
        "\n是否保存触觉传感器点云数据? (触觉点云数据会占用较大存储空间)",
        {"1": "是，保存触觉点云数据", "2": "否，不保存触觉点云数据"}
    )
    save_tactile_pointcloud = (save_pc_choice == "1")
    
    # 获取用户对实时显示的选择
    show_tactile_choice = get_user_choice(
        "\n是否在采集过程中实时显示触觉图像?",
        {"1": "是，显示触觉图像", "2": "否，不显示触觉图像"}
    )
    show_tactile = (show_tactile_choice == "1")
    
    # 获取用户对触觉视频保存的选择
    save_video_choice = get_user_choice(
        "\n是否将触觉图像保存为视频?",
        {"1": "是，保存触觉视频", "2": "否，不保存触觉视频"}
    )
    save_tactile_video = (save_video_choice == "1")
    
    # 获取用户对深度图保存的选择
    save_depth_choice = get_user_choice(
        "\n是否保存Realsense RGB-D深度图数据?",
        {"1": "是，保存RGB对齐的深度图", "2": "否，不保存深度图"}
    )
    save_depth = (save_depth_choice == "1")
    
    # 获取用户对Realsense点云保存的选择
    save_realsense_pointcloud_choice = get_user_choice(
        "\n是否保存Realsense点云数据? (0: 保存无颜色点云, 1: 保存XYZRGB带颜色点云, 2: 不保存点云)",
        {"0": "保存无颜色点云", "1": "保存XYZRGB带颜色点云", "2": "不保存点云"}
    )
    save_realsense_pointcloud = int(save_realsense_pointcloud_choice)
    print("========-----初始化---连接设备中----========")
    
    # 初始化数据采集器
    collector = DataCollector(freq=freq, max_buffer_size=max_buffer_size, 
                              save_dir=save_dir,
                              save_tactile_pointcloud=save_tactile_pointcloud, 
                              show_tactile=show_tactile,
                              save_tactile_video=save_tactile_video,
                              save_depth=save_depth,
                              save_realsense_pointcloud=save_realsense_pointcloud)
    
    print("=========================================")
    print("    机械臂遥操作数据采集程序 (含触觉)   ")
    print("=========================================")
    print("空格键开始采集，回车键结束该条数据采集")
    print("结束后可再次按空格键开始新一条数据采集")
    print("按Ctrl+C退出整个采集程序")
    print(f"采集频率设置为{freq}Hz，数据将保存为HDF5格式")
    print(f"缓冲区限制: {max_buffer_size}帧 (约{max_buffer_size/freq:.1f}秒数据)")
    print(f"数据保存目录: {os.path.abspath(save_dir)}")
    print(f"保存触觉点云: {'是' if save_tactile_pointcloud else '否'}")
    print(f"保存Realsense深度图: {'是' if save_depth else '否'}")
    print(f"显示触觉图像: {'是' if show_tactile else '否'}")
    print(f"保存触觉视频: {'是' if save_tactile_video else '否'}")
    if save_realsense_pointcloud == 0:
        print("保存Realsense点云: 是 (无颜色)")
    elif save_realsense_pointcloud == 1:
        print("保存Realsense点云: 是 (有颜色)")
    elif save_realsense_pointcloud == 2:
        print("不保存Realsense点云")
    
    try:
        while True:
            # 等待空格键开始采集
            print("\n按空格键开始采集新的一条数据...")
            wait_for_space()
            
            # 开始采集
            collector.start_collecting()
            
            # 等待回车键结束采集
            wait_for_enter_to_stop()
            
            # 停止采集（会自动保存数据）
            collector.stop_collecting()
            
            print("\n该条数据采集完成，数据已保存")
            
    except KeyboardInterrupt:
        print("\n检测到键盘中断(Ctrl+C)，正在安全退出...")
    finally:
        collector.exit()
        print("程序已安全退出")

if __name__ == "__main__":
    main() 