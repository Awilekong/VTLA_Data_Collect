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
import copy

# === 导入本地 GelSight SDK ===
# 添加本地gelsight路径到系统路径，使用相对路径
script_dir = os.path.dirname(os.path.abspath(__file__))
# 向上一级目录查找gsrobotics文件夹
gelsight_path = os.path.join(os.path.dirname(script_dir), "gsrobotics")
if gelsight_path not in sys.path:
    sys.path.insert(0, gelsight_path)

# 现在导入本地gelsight模块
from gelmini import gsdevice
from gelmini.gs3drecon import Reconstruction3D, Visualize3D
from gelmini import find_marker
from gelmini import marker_detection
from gelmini import setting

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
                 max_buffer_size=1000, save_pointcloud=True, show_tactile=False,
                 save_tactile_video=False):
        """
        初始化数据采集器
        Args:
            freq: 采集频率 (Hz)
            save_dir: 数据保存目录
            max_buffer_size: 数据缓冲区最大帧数，超过此值会触发自动保存
            save_pointcloud: 是否保存触觉点云数据
            show_tactile: 是否在采集过程中显示触觉图像
            save_tactile_video: 是否将触觉图像保存为视频
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
        self.save_pointcloud = save_pointcloud  # 新增：是否保存点云数据
        self.show_tactile = show_tactile  # 新增：是否显示触觉图像
        self.save_tactile_video = save_tactile_video  # 新增：是否保存触觉视频
        # 视频编码器和视频输出对象（分别为两个传感器）
        self.video_writer1 = None  # 第一个传感器的视频写入器
        self.video_writer2 = None  # 第二个传感器的视频写入器
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用MP4格式
        # 记录时间戳，用于命名视频文件
        self.video_timestamp = None
        # 触觉视频保存目录
        self.video_dir = os.path.join(self.save_dir, "tactile_videos")
        if self.save_tactile_video:
            os.makedirs(self.video_dir, exist_ok=True)
        
        # 初始化数据缓冲区
        self.data_buffer = {
            "timestamps": [],
            "observation.images.image": [],
            "observation.images.wrist_image": [],
            "observation.state": [],
            "observation.ee_pose": [],
            "action": [],
            "observation.tactile.img1": [],
            "observation.tactile.img2": [],
        }
        
        # 如果保存点云，添加点云数据缓冲区
        if self.save_pointcloud:
            self.data_buffer["observation.tactile.pc1"] = []
            self.data_buffer["observation.tactile.pc2"] = []
        
        # 初始化机器人和夹爪
        self.robot_ip = "192.168.1.10"     # 机械臂和夹爪的IP地址------修改为你的实际IP地址
        self.gripper_ip = "192.168.1.10"   # 机械臂和夹爪的IP地址------修改为你的实际IP地址
        # self.robot_ip = "localhost"
        # self.gripper_ip = "localhost"
        self.robot = RobotInterface(ip_address=self.robot_ip, enforce_version=False)
        # ---------修改默认的roboteinterface后，要加上enforce_version=False参数-------- 
        self.gripper = GripperInterface(ip_address=self.gripper_ip)
        
        try:
            print("初始化 GelSight 传感器1")
            # 使用直接的摄像头ID而不是名称，避免重复连接到同一设备
            self.gel1 = gsdevice.Camera(0)

            self.gel1.connect()
        except Exception as e:
            print(f"错误: 无法连接第一个 GelSight 传感器: {e}")
            sys.exit(1)
        
        try:
            print("初始化 GelSight 传感器2")
            self.gel2 = gsdevice.Camera("GelSight Mini")

            self.gel2.connect()
        except Exception as e:
            print(f"错误: 无法连接第二个 GelSight 传感器: {e}")
            print("两个传感器都必须连接才能继续，程序将退出")
            # 关闭已连接的传感器，释放摄像头资源
            if hasattr(self.gel1, 'cam') and self.gel1.cam is not None:
                self.gel1.cam.release()
            sys.exit(1)
        
        print("GelSight传感器初始化完成")
        
        # === 初始化标记点探测和匹配 ===
        print("初始化标记点探测和匹配")
        # 抓取初始帧用于标记点匹配
        self.frame1_0 = self.gel1.get_image()
        self.frame2_0 = self.gel2.get_image()
        
        if self.frame1_0 is None or self.frame2_0 is None:
            print("警告：无法获取GelSight初始帧，请检查相机连接")
            print("请确保没有其他程序正在使用GelSight相机")
            sys.exit(1)
        
        # === 新增：初始化形变场跟踪所需的变量和设置 ===
        setting.init()
        
        # 获取初始帧并进行标记点检测 - 用于初始化形变场计算
        print("正在初始化标记点检测...")
        # 第一个传感器
        for i in range(50):  # 清除黑帧
            frame1 = self.gel1.get_image()
            if i == 48:
                mask1 = marker_detection.find_marker(frame1)
                mc1 = marker_detection.marker_center(mask1, frame1)
        
        # 第二个传感器
        for i in range(50):  # 清除黑帧
            frame2 = self.gel2.get_image()
            if i == 48:
                mask2 = marker_detection.find_marker(frame2)
                mc2 = marker_detection.marker_center(mask2, frame2)
        
        # 排序标记点（第一个传感器）
        mc1_sorted_row = mc1[mc1[:,0].argsort()]
        mc1_row = mc1_sorted_row[:setting.N_]
        mc1_row = mc1_row[mc1_row[:,1].argsort()]
        
        mc1_sorted_col = mc1[mc1[:,1].argsort()]
        mc1_col = mc1_sorted_col[:setting.M_]
        mc1_col = mc1_col[mc1_col[:,0].argsort()]
        
        # 排序标记点（第二个传感器）
        mc2_sorted_row = mc2[mc2[:,0].argsort()]
        mc2_row = mc2_sorted_row[:setting.N_]
        mc2_row = mc2_row[mc2_row[:,1].argsort()]
        
        mc2_sorted_col = mc2[mc2[:,1].argsort()]
        mc2_col = mc2_sorted_col[:setting.M_]
        mc2_col = mc2_col[mc2_col[:,0].argsort()]
        
        # 获取标记点网格参数
        self.N_ = setting.N_
        self.M_ = setting.M_
        fps_ = setting.fps_
        
        # 第一个传感器的标记点参数
        x0_1 = np.round(mc1_row[0][0])
        y0_1 = np.round(mc1_row[0][1])
        dx_1 = mc1_col[1, 0] - mc1_col[0, 0]
        dy_1 = mc1_row[1, 1] - mc1_row[0, 1]
        
        # 第二个传感器的标记点参数
        x0_2 = np.round(mc2_row[0][0])
        y0_2 = np.round(mc2_row[0][1])
        dx_2 = mc2_col[1, 0] - mc2_col[0, 0]
        dy_2 = mc2_row[1, 1] - mc2_row[0, 1]
        
        print(f'传感器1 - x0: {x0_1}, y0: {y0_1}, dx: {dx_1}, dy: {dy_1}')
        print(f'传感器2 - x0: {x0_2}, y0: {y0_2}, dx: {dx_2}, dy: {dy_2}')
        
        # 创建标记点匹配对象
        self.matcher1 = find_marker.Matching(self.N_, self.M_, fps_, x0_1, y0_1, dx_1, dy_1)
        self.matcher2 = find_marker.Matching(self.N_, self.M_, fps_, x0_2, y0_2, dx_2, dy_2)
        
        # 存储初始帧用于比较
        self.frame1_0 = frame1.copy()
        self.frame1_0 = cv2.GaussianBlur(self.frame1_0, (int(63), int(63)), 0)
        
        self.frame2_0 = frame2.copy()
        self.frame2_0 = cv2.GaussianBlur(self.frame2_0, (int(63), int(63)), 0)
        
        # 初始化显示窗口
        if self.show_tactile:
            cv2.namedWindow('GelSight 形变场', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('GelSight 形变场', 320*6, 240*3)  # 左右拼接，宽度翻倍
        
        # === 新增：初始化点云重建模型 ===
        if self.save_pointcloud:
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
        self.global_cam_pipeline.start(global_config)
        
        # 初始化腕部相机
        print("初始化腕部相机")
        self.wrist_cam_pipeline = rs.pipeline()
        wrist_config = rs.config()
        # 使用设备序列号区分腕部相机
        wrist_config.enable_device(self.serial_2)  # 请替换为实际腕部相机的序列号
        wrist_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.wrist_cam_pipeline.start(wrist_config)
    
    def _signal_handler(self, sig, frame):
        """处理中断信号，确保数据完整性"""
        print("\n正在安全停止数据采集...")
        self.stop_collecting()
        
    def _get_global_camera_image(self):
        """获取全局相机图像
        
        Returns:
            numpy.ndarray: 形状为(256, 256, 3)的RGB图像
        """
        frames = self.global_cam_pipeline.wait_for_frames()
        # 获取彩色图像帧
        color_frame = frames.get_color_frame()
        
        # 转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())
        
        # 将图像缩放到256x256
        color_image = cv2.resize(color_image, (256, 256))
        
        # 转换为RGB格式
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        return color_image
    
    def _get_wrist_camera_image(self):
        """获取腕部相机(Realsense)图像
        
        Returns:
            numpy.ndarray: 形状为(256, 256, 3)的RGB图像
        """
        frames = self.wrist_cam_pipeline.wait_for_frames()
        # 获取彩色图像帧
        color_frame = frames.get_color_frame()
        
        # 转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())
        
        # 将图像缩放到256x256
        color_image = cv2.resize(color_image, (256, 256))
        
        # 转换为RGB格式
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        return color_image
    
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
    
    def _collect_data_point(self):
        """采集一帧数据"""
        # 检查缓冲区大小，如果超过最大限制，自动保存数据
        if len(self.data_buffer["timestamps"]) >= self.max_buffer_size:
            print(f"\n缓冲区已达到最大限制({self.max_buffer_size}帧)，正在自动保存数据...")
            self._save_data()
            print("数据已保存，继续采集...")
            
        global_img = self._get_global_camera_image()
        wrist_img = self._get_wrist_camera_image()
        robot_state = self._get_robot_state()
        robot_ee_pose = self._get_robot_ee_pose()
        robot_desired_action = self._get_robot_desired_action()
        timestamp = time.time()
        
        # === 采集触觉图像和点云 ===
        # 采集两个传感器的数据
        tac_img1 = self.gel1.get_image()
        tac_img2 = self.gel2.get_image()
        
        # === 修改：计算形变场并显示 ===
        if tac_img1 is not None and tac_img2 is not None and self.show_tactile:
            # 获取当前帧计数
            frame_count = len(self.data_buffer["timestamps"])
            
            # 复制原始图像用于显示
            display1 = copy.deepcopy(tac_img1)
            display2 = copy.deepcopy(tac_img2)
            
            # 检测标记点
            mask1 = marker_detection.find_marker(tac_img1)
            mc1 = marker_detection.marker_center(mask1, tac_img1)
            
            mask2 = marker_detection.find_marker(tac_img2)
            mc2 = marker_detection.marker_center(mask2, tac_img2)
            
            # 匹配和跟踪标记点
            self.matcher1.init(mc1)
            self.matcher1.run()
            flow1 = self.matcher1.get_flow()
            
            self.matcher2.init(mc2)
            self.matcher2.run()
            flow2 = self.matcher2.get_flow()
            marker_motion1 = np.zeros((self.M_ * self.N_, 2))
            marker_motion2 = np.zeros((self.M_ * self.N_, 2))
            k=0
            for i in range(self.M_):
                for j in range(self.N_):
                    marker_motion1[k] = [flow1[2][i][j] - flow1[0][i][j], flow1[3][i][j] - flow1[1][i][j]]
                    k += 1
            k=0
            for i in range(self.M_):
                for j in range(self.N_):
                    marker_motion2[k] = [flow2[2][i][j] - flow2[0][i][j], flow2[3][i][j] - flow2[1][i][j]]
                    k += 1
            # 绘制形变场
            marker_detection.draw_flow(display1, flow1)
            marker_detection.draw_flow(display2, flow2)
            
            # 调整图像大小以便更好地显示
            display1 = cv2.resize(display1, (display1.shape[1]*3, display1.shape[0]*3))
            display2 = cv2.resize(display2, (display2.shape[1]*3, display2.shape[0]*3))
            
            # 添加标签以区分两个传感器
            cv2.putText(display1, f"传感器 1 | 帧: {frame_count}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                      0.8, (0, 255, 0), 2)
            cv2.putText(display2, f"传感器 2 | 帧: {frame_count}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                      0.8, (0, 255, 0), 2)
            
            # 确保两个图像尺寸一致
            if display1.shape[0] != display2.shape[0]:
                # 获取较大的高度
                max_height = max(display1.shape[0], display2.shape[0])
                # 调整高度较小的图像
                if display1.shape[0] < max_height:
                    display1 = cv2.copyMakeBorder(display1, 0, max_height - display1.shape[0], 
                                                0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
                else:
                    display2 = cv2.copyMakeBorder(display2, 0, max_height - display2.shape[0], 
                                                0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
            
            # 水平拼接两个图像
            combined_display = np.hstack((display1, display2))
            
            # 显示拼接后的结果
            cv2.imshow('GelSight 形变场', combined_display)
            cv2.waitKey(1)
            
        # 处理视频保存（两个独立视频）
        if self.save_tactile_video:
            # 初始化视频写入器（如果需要）
            if self.video_timestamp is None:
                self.video_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                video_path1 = os.path.join(self.video_dir, f"tactile1_{self.video_timestamp}.mp4")
                video_path2 = os.path.join(self.video_dir, f"tactile2_{self.video_timestamp}.mp4")
                
                # 获取图像尺寸
                h1, w1 = tac_img1.shape[:2] if tac_img1 is not None else (240, 320)
                h2, w2 = tac_img2.shape[:2] if tac_img2 is not None else (240, 320)
                
                # 创建视频写入器
                self.video_writer1 = cv2.VideoWriter(video_path1, self.fourcc, self.freq, (w1, h1))
                self.video_writer2 = cv2.VideoWriter(video_path2, self.fourcc, self.freq, (w2, h2))
                
                print(f"已开始录制触觉视频，保存到: {os.path.relpath(self.video_dir)}")
            
            # 将当前帧写入视频
            if tac_img1 is not None and self.video_writer1 is not None:
                self.video_writer1.write(tac_img1)
                
            if tac_img2 is not None and self.video_writer2 is not None:
                self.video_writer2.write(tac_img2)
        
        # 根据配置决定是否采集点云
        if self.save_pointcloud:
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
            if self.save_pointcloud:
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
            # 等待
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
    freq = 10
    max_buffer_size = 500
    
    # 设置默认保存目录
    save_dir = "./collected_data"
    
    # 获取用户对点云保存的选择
    save_pc_choice = get_user_choice(
        "\n是否保存触觉点云数据? (点云数据会占用较大存储空间)",
        {"1": "是，保存点云数据", "2": "否，不保存点云数据"}
    )
    save_pointcloud = (save_pc_choice == "1")
    
    # 获取用户对实时显示的选择
    show_tactile_choice = get_user_choice(
        "\n是否在采集过程中实时显示触觉形变场?",
        {"1": "是，显示触觉形变场", "2": "否，不显示触觉形变场"}
    )
    show_tactile = (show_tactile_choice == "1")
    
    # 获取用户对触觉视频保存的选择
    save_video_choice = get_user_choice(
        "\n是否将触觉图像保存为视频?",
        {"1": "是，保存触觉视频", "2": "否，不保存触觉视频"}
    )
    save_tactile_video = (save_video_choice == "1")
    
    # 初始化数据采集器
    collector = DataCollector(freq=freq, max_buffer_size=max_buffer_size, 
                              save_dir=save_dir,
                              save_pointcloud=save_pointcloud, 
                              show_tactile=show_tactile,
                              save_tactile_video=save_tactile_video)
    
    print("=========================================")
    print("    机械臂遥操作数据采集程序 (含触觉)   ")
    print("=========================================")
    print("空格键开始采集，回车键结束该条数据采集")
    print("结束后可再次按空格键开始新一条数据采集")
    print("按Ctrl+C退出整个采集程序")
    print(f"采集频率设置为{freq}Hz，数据将保存为HDF5格式")
    print(f"缓冲区限制: {max_buffer_size}帧 (约{max_buffer_size/freq:.1f}秒数据)")
    print(f"数据保存目录: {os.path.abspath(save_dir)}")
    print(f"保存点云: {'是' if save_pointcloud else '否'}")
    print(f"显示触觉形变场: {'是' if show_tactile else '否'}")
    print(f"保存触觉视频: {'是' if save_tactile_video else '否'}")
    
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