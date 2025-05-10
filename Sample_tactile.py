import time
import h5py
import numpy as np
import cv2
import threading
import os
from datetime import datetime
import signal
import sys
sys.path.append('/home/robot/01_GELLO/gello_software')
# from third_party.robotiq_2finger_grippers.robotiq_2f_gripper import Robotiq2FingerGripper
import pyrealsense2 as rs
from polymetis import RobotInterface, GripperInterface

from gello.robots.robot import Robot

# === 新增：导入官方 GelSight SDK ===
# 需提前 pip install gsrobotics 并设置 PYTHONPATH
from gelsight import gsdevice  # === 新增：GelSight 接口库 ===
from gelsight.gs3drecon import Reconstruction3D

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
        self.serial_1 = "819612073036"  # 全局相机序列号
        self.serial_2 = "819612071794"  # 腕部相机序列号
        self.freq = freq
        self.period = 1.0 / freq
        self.save_dir = save_dir
        self.is_collecting = False
        self.exit_flag = False

        # === 新增：在 data_buffer 中添加 GelSight 数据存储键 ===
        self.data_buffer = {
            "observation.images.image": [],
            "observation.images.wrist_image": [],
            "observation.state": [],
            "action": [],
            "timestamps": [],
            "tactile_img1": [],       # === 新增：触觉图像 1 ===
            "tactile_img2": [],       # === 新增：触觉图像 2 ===
            "tactile_pc1": [],        # === 新增：触觉点云 1 ===
            "tactile_pc2": []         # === 新增：触觉点云 2 ===
        }

        # 机械臂与夹爪接口
        self.robot = RobotInterface(ip_address="localhost")  # 请替换为实际 IP
        self.gripper = GripperInterface(comport="/dev/ttyUSB1")

        # === 初始化两路 GelSight 传感器实例 ===
        self.gel1 = gsdevice.Camera("0")  # 设备 ID 根据系统调整
        self.gel1.connect()             
        # self.gel2 = gsdevice.Camera("1")
        # self.gel2.connect()

        self.mmpp = 0.0634
        script_dir = os.path.dirname(os.path.abspath(__file__))
        net_path = os.path.join(script_dir, "nnmini.pt")
        if not os.path.isfile(net_path):
            raise FileNotFoundError(f"找不到模型 nnmini.pt，请放在：{script_dir}")
        
        # 路径相同、模型相同，但需要两个实例
        self.reconstructor1 = Reconstruction3D(self.gel1)
        self.reconstructor1.load_nn(net_path, "cpu")
        # self.reconstructor2 = Reconstruction3D(self.gel2)
        # self.reconstructor2.load_nn(net_path, "cpu")

        # 添加：点云可视化器（用于保存点云）
        save_path_1 = os.path.join(save_dir, "pointclouds_1")
        # save_path_2 = os.path.join(save_dir, "pointclouds_2")
        os.makedirs(save_path, exist_ok=True)  # 确保保存路径存在
        self.visualizer1 = gs3drecon.Visualize3D(self.gel1.imgh, self.gel1.imgw, '', self.mmpp)
        # self.visualizer2 = gs3drecon.Visualize3D(self.gel2.imgh, self.gel2.imgw, '', self.mmpp)

        # 确保存储目录存在
        os.makedirs(save_dir, exist_ok=True)
        
        # 设置信号处理，确保优雅退出
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 初始化全局相机
        self.global_cam_pipeline = rs.pipeline()
        global_config = rs.config()
        global_config.enable_device(self.serial_1)
        global_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.global_cam_pipeline.start(global_config)
        
        # 初始化腕部相机
        self.wrist_cam_pipeline = rs.pipeline()
        wrist_config = rs.config()
        wrist_config.enable_device(self.serial_2)
        wrist_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.wrist_cam_pipeline.start(wrist_config)
        
        if not USE_READCHAR:
            self.esc_pressed = False
            self.keyboard_listener = None
        
    def _signal_handler(self, sig, frame):
        """处理中断信号，确保数据完整性"""
        print("\n正在安全停止数据采集...")
        self.stop_collecting()
        
    def _get_global_camera_image(self) -> np.ndarray:
        """获取全局相机图像"""
        frames = self.global_cam_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.resize(color_image, (256, 256))
        return cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    
    def _get_wrist_camera_image(self):
        """获取腕部相机图像"""
        frames = self.wrist_cam_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.resize(color_image, (256, 256))
        return cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    
    def _get_robot_state(self):
        """获取机械臂状态（关节角 + 夹爪宽度）"""
        joint_state = self.robot.get_joint_positions()
        gripper_width = self.gripper.get_pos()
        return np.concatenate([joint_state, np.array([gripper_width])]).astype(np.float32)
    
    def _get_robot_action(self):
        """获取机械臂末端执行器位置与四元数"""
        pos, quat = self.robot.get_ee_pose()
        return np.concatenate([pos, quat]).astype(np.float32)
    
    # === 新增：获取 GelSight 点云 ===
    def _get_gelsight_pointcloud(self, sensor):
        """
        用神经网络+Poisson 求解深度，生成 N×3 点云
        """
        # 1) 先拿 2D 图
        img = sensor.get_image()
        if img is None:
            print("WARNING: 无法从 GelSight 读取图像，返回空点云")
            return np.empty((0, 3))

        # 2) 选用对应的 reconstructor
        if sensor is self.gel1:
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
    
    def _on_key_press(self, key):
        """pynput 键盘按下回调"""
        if hasattr(key, 'char') and key.char == '\x1b':
            self.esc_pressed = True
            return False
        return True
        
    def _check_esc_pressed(self):
        """检查是否按下 ESC 键"""
        if USE_READCHAR:
            import sys, select
            if select.select([sys.stdin], [], [], 0)[0]:
                return readchar.readchar() == readchar.key.ESC
            return False
        else:
            return self.esc_pressed
    
    def _collect_data_point(self):
        """采集一帧数据"""
        global_img    = self._get_global_camera_image()
        wrist_img     = self._get_wrist_camera_image()
        robot_state   = self._get_robot_state()
        robot_action  = self._get_robot_action()
        timestamp     = time.time()
        
        # === 新增：采集两路触觉图像 ===
        tac_img1 = self.gel1.get_image()      # get_image() 获取 2D 彩色图像:contentReference[oaicite:11]{index=11}
        # tac_img2 = self.gel2.get_image()
        # === 新增：采集两路触觉点云 ===
        tac_pc1  = self._get_gelsight_pointcloud(self.gel1)
        # tac_pc2  = self._get_gelsight_pointcloud(self.gel2)
        
        # 添加到缓冲区
        self.data_buffer["observation.images.image"].append(global_img)
        self.data_buffer["observation.images.wrist_image"].append(wrist_img)
        self.data_buffer["observation.state"].append(robot_state)
        self.data_buffer["action"].append(robot_action)
        self.data_buffer["timestamps"].append(timestamp)
        self.data_buffer["tactile_img1"].append(tac_img1)
        # self.data_buffer["tactile_img2"].append(tac_img2)
        self.data_buffer["tactile_pc1"].append(tac_pc1)
        # self.data_buffer["tactile_pc2"].append(tac_pc2)
        
        return timestamp
    
    def _collection_loop(self):
        """数据采集主循环"""
        self.is_collecting = True
        print("开始数据采集，按ESC键结束采集...")
        if not USE_READCHAR:
            self.esc_pressed = False
            self.keyboard_listener = kb.Listener(on_press=self._on_key_press)
            self.keyboard_listener.start()
        
        count = 0
        start_time = time.time()
        last_time = start_time
        
        while self.is_collecting and not self.exit_flag:
            loop_start = time.time()
            
            timestamp = self._collect_data_point()
            count += 1
            
            if count % 10 == 0:
                current_time = time.time()
                freq = 10 / (current_time - last_time) if current_time > last_time else 0
                print(f"\r当前采集频率: {freq:.2f} Hz, 已采集帧数: {count}", end="")
                last_time = current_time
            
            if self._check_esc_pressed():
                print("\n检测到ESC键，停止采集...")
                self.is_collecting = False
                break
                
            elapsed = time.time() - loop_start
            time.sleep(max(0, self.period - elapsed))
        
        if not USE_READCHAR and self.keyboard_listener is not None:
            self.keyboard_listener.stop()
        
        total_time = time.time() - start_time
        avg_freq = count / total_time if total_time > 0 else 0
        print(f"\n采集结束，共采集 {count} 帧，平均采集频率: {avg_freq:.2f} Hz")
        
        # 采集结束后保存数据
        self._save_data()
    
    def _save_data(self):
        """将采集的数据保存为 TXT、JPEG 和 NumPy Binary 格式"""
        if not self.data_buffer["timestamps"]:
            print("没有数据需要保存")
            return
            
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_folder = os.path.join(self.save_dir, f"pick_right_{timestamp_str}")
        
        # 创建保存目录
        os.makedirs(save_folder, exist_ok=True)
        for sub in ["global_images","wrist_images","joint_states","actions",
                    "tactile_img1","tactile_img2"]:
            os.makedirs(os.path.join(save_folder, sub), exist_ok=True)
        
        print(f"正在保存数据到 {save_folder}...")
        
        # 保存时间戳
        with open(os.path.join(save_folder, "timestamps.txt"), 'w') as f:
            for idx, ts in enumerate(self.data_buffer["timestamps"]):
                f.write(f"{idx},{ts}\n")
        
        # 保存图像与状态
        for idx, img in enumerate(self.data_buffer["observation.images.image"]):
            path = os.path.join(save_folder, "global_images", f"{idx:06d}.jpeg")
            cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        for idx, img in enumerate(self.data_buffer["observation.images.wrist_image"]):
            path = os.path.join(save_folder, "wrist_images", f"{idx:06d}.jpeg")
            cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        for idx, st in enumerate(self.data_buffer["observation.state"]):
            with open(os.path.join(save_folder, "joint_states", f"{idx:06d}.txt"), 'w') as f:
                f.write(','.join(map(str, st)))
        for idx, ac in enumerate(self.data_buffer["action"]):
            with open(os.path.join(save_folder, "actions", f"{idx:06d}.txt"), 'w') as f:
                f.write(','.join(map(str, ac)))
        
        # === 新增：保存触觉图像 ===
        for idx, img in enumerate(self.data_buffer["tactile_img1"]):
            cv2.imwrite(os.path.join(save_folder, "tactile_img1", f"{idx:06d}.jpeg"), img)
        for idx, img in enumerate(self.data_buffer["tactile_img2"]):
            cv2.imwrite(os.path.join(save_folder, "tactile_img2", f"{idx:06d}.jpeg"), img)
        
        # === 新增：保存触觉点云为 NumPy 文件 ===
        for idx, pc in enumerate(self.data_buffer["tactile_pc1"]):
            np.save(os.path.join(save_folder, "tactile_pc1", f"{idx:06d}.npy"), pc)
        for idx, pc in enumerate(self.data_buffer["tactile_pc2"]):
            np.save(os.path.join(save_folder, "tactile_pc2", f"{idx:06d}.npy"), pc)
        
        print(f"数据成功保存到 {save_folder}")
        
        # 清空缓冲区
        for key in self.data_buffer:
            self.data_buffer[key] = []
    
    def start_collecting(self):
        """开始数据采集"""
        if self.is_collecting:
            print("数据采集已经在进行中")
            return
            
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
        
        if hasattr(self, 'global_cam_pipeline'):
            self.global_cam_pipeline.stop()
        if hasattr(self, 'wrist_cam_pipeline'):
            self.wrist_cam_pipeline.stop()
    
    def __del__(self):
        """析构函数，确保资源被释放"""
        if hasattr(self, 'global_cam_pipeline'):
            self.global_cam_pipeline.stop()
        if hasattr(self, 'wrist_cam_pipeline'):
            self.wrist_cam_pipeline.stop()

def wait_for_enter():
    """等待用户按下回车键"""
    print("按回车键开始数据采集...")
    if USE_READCHAR:
        while True:
            if readchar.readchar() == readchar.key.ENTER:
                break
    else:
        enter_pressed = [False]
        def on_press(key):
            if key == kb.Key.enter:
                enter_pressed[0] = True
                return False
            return True
        with kb.Listener(on_press=on_press):
            while not enter_pressed[0]:
                time.sleep(0.1)

def main():
    """主函数"""
    collector = DataCollector(freq=10)
    print("=========================================")
    print("       机械臂遥操作数据采集程序")
    print("=========================================")
    print("按回车键开始数据采集，采集过程中按ESC键结束")
    print("采集频率设置为10Hz，数据将保存到本地文件夹")
    try:
        wait_for_enter()
        collector.start_collecting()
        while collector.is_collecting:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n检测到键盘中断，正在安全退出...")
    finally:
        collector.exit()
        print("程序已安全退出")

if __name__ == "__main__":
    main()