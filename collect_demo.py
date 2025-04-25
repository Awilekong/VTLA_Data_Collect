#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
    
    def __init__(self, freq=10, save_dir="collected_data_right"):
        """
        初始化数据采集器
        
        Args:
            freq: 采集频率 (Hz)
            save_dir: 数据保存目录
        """
        self.serial_1 = "136622074722"  # 全局相机序列号
        self.serial_2 = "233622071355"  # 腕部相机序列号
        self.freq = freq
        self.period = 1.0 / freq
        self.save_dir = save_dir
        self.is_collecting = False
        self.exit_flag = False
        self.data_buffer = {
            "observation.images.image": [],
            "observation.images.wrist_image": [],
            "observation.state": [],
            "action": [],
            "timestamps": []
        }
        self.robot = RobotInterface(ip_address="192.168.1.100")
        self.gripper = GripperInterface(ip_address="192.168.1.100")
        
        # 确保存储目录存在
        os.makedirs(save_dir, exist_ok=True)
        
        # 设置信号处理，确保优雅退出
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 初始化全局相机
        self.global_cam_pipeline = rs.pipeline()
        global_config = rs.config()
        # 使用设备序列号区分全局相机
        global_config.enable_device(self.serial_1)  # 请替换为实际全局相机的序列号
        global_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.global_cam_pipeline.start(global_config)
        
        # 初始化腕部相机
        self.wrist_cam_pipeline = rs.pipeline()
        wrist_config = rs.config()
        # 使用设备序列号区分腕部相机
        wrist_config.enable_device(self.serial_2)  # 请替换为实际腕部相机的序列号
        wrist_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.wrist_cam_pipeline.start(wrist_config)
        
        # 用于监听ESC键的变量
        if not USE_READCHAR:
            self.esc_pressed = False
            self.keyboard_listener = None
        
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
    
    def _get_robot_action(self):
        """获取机械臂末端位姿(3位置+4四元数)
        
        Returns:
            numpy.ndarray: 形状为(7,)的浮点数数组，包含3个位置和4个四元数
        """
        pos, quat = self.robot.get_ee_pose()
        # 确保四元数归一化
        # quat = np.random.uniform(-1, 1, (4,))
        # quat = quat / np.linalg.norm(quat)
        
        return np.concatenate([pos, quat]).astype(np.float32)
    
    def _on_key_press(self, key):
        """pynput键盘按下回调函数"""
        if hasattr(key, 'char') and key.char == '\x1b':  # ESC键
            self.esc_pressed = True
            return False  # 停止监听
        return True
        
    def _check_esc_pressed(self):
        """检查是否按下ESC键"""
        if USE_READCHAR:
            # 使用readchar的非阻塞读取方式
            import sys, select
            if select.select([sys.stdin], [], [], 0)[0]:
                char = readchar.readchar()
                return char == readchar.key.ESC
            return False
        else:
            # 使用pynput的标志变量
            return self.esc_pressed
    
    def _collect_data_point(self):
        """采集一帧数据"""
        global_img = self._get_global_camera_image()
        wrist_img = self._get_wrist_camera_image()
        robot_state = self._get_robot_state()
        robot_action = self._get_robot_action()
        timestamp = time.time()
        
        # 添加到缓冲区
        self.data_buffer["observation.images.image"].append(global_img)
        self.data_buffer["observation.images.wrist_image"].append(wrist_img)
        self.data_buffer["observation.state"].append(robot_state)
        self.data_buffer["action"].append(robot_action)
        self.data_buffer["timestamps"].append(timestamp)
        
        return timestamp
    
    def _collection_loop(self):
        """数据采集主循环"""
        self.is_collecting = True
        print("开始数据采集，按ESC键结束采集...")
        
        # 如果使用pynput，启动键盘监听
        if not USE_READCHAR:
            self.esc_pressed = False
            self.keyboard_listener = kb.Listener(on_press=self._on_key_press)
            self.keyboard_listener.start()
        
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
            
            # 检查是否按ESC键停止采集
            if self._check_esc_pressed():
                print("\n检测到ESC键，停止采集...")
                self.is_collecting = False
                break
                
            # 精确控制采集频率
            elapsed = time.time() - loop_start
            sleep_time = max(0, self.period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 停止键盘监听（如果有的话）
        if not USE_READCHAR and self.keyboard_listener is not None:
            self.keyboard_listener.stop()
        
        total_time = time.time() - start_time
        avg_freq = count / total_time if total_time > 0 else 0
        print(f"\n采集结束，共采集 {count} 帧数据，平均采集频率: {avg_freq:.2f} Hz")
        
        # 采集结束后保存数据
        self._save_data()
    
    def _save_data(self):
        """将采集的数据保存为HDF5格式"""
        if not self.data_buffer["timestamps"]:
            print("没有数据需要保存")
            return
            
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.save_dir, f"pick_right_{timestamp_str}.h5")
        
        print(f"正在保存数据到 {filename}...")
        
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
            f.create_dataset("action", 
                            data=np.array(self.data_buffer["action"]),
                            dtype='float32')
            
        print(f"数据成功保存到 {filename}")
        
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
    
    def exit(self):
        """安全退出程序"""
        self.exit_flag = True
        self.stop_collecting()
        
        # 关闭相机流
        if hasattr(self, 'global_cam_pipeline'):
            self.global_cam_pipeline.stop()
        if hasattr(self, 'wrist_cam_pipeline'):
            self.wrist_cam_pipeline.stop()
    
    def __del__(self):
        """析构函数，确保资源被释放"""
        # 关闭相机流
        if hasattr(self, 'global_cam_pipeline'):
            self.global_cam_pipeline.stop()
        if hasattr(self, 'wrist_cam_pipeline'):
            self.wrist_cam_pipeline.stop()

def wait_for_enter():
    """等待用户按下回车键"""
    print("按回车键开始数据采集...")
    if USE_READCHAR:
        while True:
            key = readchar.readchar()
            if key == readchar.key.ENTER:
                break
    else:
        # 使用pynput监听回车键
        enter_pressed = [False]
        
        def on_press(key):
            if key == kb.Key.enter:
                enter_pressed[0] = True
                return False
            return True
        
        with kb.Listener(on_press=on_press) as listener:
            while not enter_pressed[0]:
                time.sleep(0.1)
            listener.stop()

def main():
    """主函数"""
    collector = DataCollector(freq=10)
    
    print("=========================================")
    print("       机械臂遥操作数据采集程序")
    print("=========================================")
    print("按回车键开始数据采集，采集过程中按ESC键结束")
    print("采集频率设置为10Hz，数据将保存为HDF5格式")
    
    try:
        wait_for_enter()
        collector.start_collecting()
        
        # 等待采集线程结束
        while collector.is_collecting:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n检测到键盘中断，正在安全退出...")
    finally:
        collector.exit()
        print("程序已安全退出")

if __name__ == "__main__":
    main() 