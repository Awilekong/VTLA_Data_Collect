#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
from PIL import Image, ImageDraw, ImageFont
import os

class CameraViewer:
    def __init__(self):
        # 相机序列号
        self.serial_1 = "136622074722"  # 全局相机序列号
        self.serial_2 = "233622071355"  # 腕部相机序列号
        
        # 初始化相机流配置
        self.config_1 = rs.config()
        self.config_2 = rs.config()
        
        # 检查是否能找到相机
        self.ctx = rs.context()
        self.devices = self.ctx.query_devices()
        if len(self.devices) == 0:
            print("错误: 未检测到任何RealSense相机!")
            sys.exit(1)
            
        print(f"检测到 {len(self.devices)} 个RealSense相机")
        
        # 列出所有可用的相机序列号
        print("可用相机序列号:")
        self.available_serials = []
        for dev in self.devices:
            serial = dev.get_info(rs.camera_info.serial_number)
            self.available_serials.append(serial)
            print(f" - {serial}")
            
        # 检查我们需要的相机是否可用
        serial1_available = self.serial_1 in self.available_serials
        serial2_available = self.serial_2 in self.available_serials
        
        if not serial1_available:
            print(f"警告: 全局相机 (序列号: {self.serial_1}) 未找到!")
            if len(self.available_serials) > 0:
                self.serial_1 = self.available_serials[0]
                print(f"使用可用相机 (序列号: {self.serial_1}) 替代")
                
        if not serial2_available:
            print(f"警告: 腕部相机 (序列号: {self.serial_2}) 未找到!")
            if len(self.available_serials) > 1:
                self.serial_2 = self.available_serials[1]
                print(f"使用可用相机 (序列号: {self.serial_2}) 替代")
            elif len(self.available_serials) == 1:
                print("只有一个相机可用，将只显示一个相机画面")
                self.serial_2 = None
            else:
                print("没有可用相机")
                sys.exit(1)
        
        # 为每个相机指定序列号
        self.config_1.enable_device(self.serial_1)
        if self.serial_2:
            self.config_2.enable_device(self.serial_2)
        
        # 只设置颜色流的分辨率和帧率
        self.config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if self.serial_2:
            self.config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 创建管道
        self.pipeline_1 = rs.pipeline()
        if self.serial_2:
            self.pipeline_2 = rs.pipeline()
            
        # 尝试查找系统字体
        self.font_path = None
        possible_font_paths = [
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",  # Ubuntu
            "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",  # Arch Linux
            "/usr/share/fonts/noto/NotoSansCJK-Regular.ttc",      # 其他 Linux 发行版
            "/usr/share/fonts/wqy-microhei/wqy-microhei.ttc",    # WQY 字体
            "/usr/share/fonts/wqy-zenhei/wqy-zenhei.ttc",
            "/usr/share/fonts/ttf-dejavu/DejaVuSans.ttf",        # 退回到非中文字体
            "/System/Library/Fonts/PingFang.ttc"                  # macOS
        ]
        
        for path in possible_font_paths:
            if os.path.exists(path):
                self.font_path = path
                print(f"找到字体: {path}")
                break
                
        if not self.font_path:
            print("警告: 未找到中文字体，将使用OpenCV内置字体")
    
    def start(self):
        """启动相机流"""
        try:
            # 启动管道
            print(f"尝试启动全局相机 (序列号: {self.serial_1})...")
            self.profile_1 = self.pipeline_1.start(self.config_1)
            print(f"成功启动全局相机 (序列号: {self.serial_1})")
            
            camera_success = True
            
            if self.serial_2:
                try:
                    print(f"尝试启动腕部相机 (序列号: {self.serial_2})...")
                    self.profile_2 = self.pipeline_2.start(self.config_2)
                    print(f"成功启动腕部相机 (序列号: {self.serial_2})")
                except Exception as e:
                    print(f"启动腕部相机出错: {e}")
                    self.serial_2 = None
            
            return camera_success
            
        except Exception as e:
            print(f"启动全局相机出错: {e}")
            return False
    
    def stop(self):
        """停止相机流"""
        try:
            self.pipeline_1.stop()
            print(f"已停止全局相机 (序列号: {self.serial_1})")
        except Exception as e:
            print(f"停止全局相机时出错: {e}")
            
        if self.serial_2:
            try:
                self.pipeline_2.stop()
                print(f"已停止腕部相机 (序列号: {self.serial_2})")
            except Exception as e:
                print(f"停止腕部相机时出错: {e}")
        
        print("已停止所有相机")
    
    def get_frames(self):
        """获取两个相机的彩色帧"""
        color_image_1 = None
        color_image_2 = None
        
        # 获取全局相机帧
        try:
            # 等待全局相机的帧
            frames_1 = self.pipeline_1.wait_for_frames(timeout_ms=1000)
            
            # 获取颜色帧
            color_frame_1 = frames_1.get_color_frame()
            
            if color_frame_1:
                # 将帧转换为numpy数组
                color_image_1 = np.asanyarray(color_frame_1.get_data())
            else:
                print("全局相机的帧数据不完整")
        except Exception as e:
            print(f"获取全局相机帧时出错: {e}")
        
        # 获取腕部相机帧
        if self.serial_2:
            try:
                # 等待腕部相机的帧
                frames_2 = self.pipeline_2.wait_for_frames(timeout_ms=1000)
                
                # 获取颜色帧
                color_frame_2 = frames_2.get_color_frame()
                
                if color_frame_2:
                    # 将帧转换为numpy数组
                    color_image_2 = np.asanyarray(color_frame_2.get_data())
                else:
                    print("腕部相机的帧数据不完整")
            except Exception as e:
                print(f"获取腕部相机帧时出错: {e}")
        
        return color_image_1, color_image_2
        
    def put_chinese_text(self, img, text, position, color, font_size=30):
        """使用PIL将中文文本绘制到图像上"""
        if self.font_path:
            # 将OpenCV图像转换为PIL图像
            pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            
            # 创建绘图对象
            draw = ImageDraw.Draw(pil_img)
            
            # 加载字体
            font = ImageFont.truetype(self.font_path, font_size)
            
            # 在图像上绘制文本
            draw.text(position, text, font=font, fill=color)
            
            # 将PIL图像转换回OpenCV格式
            return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
        else:
            # 如果没有找到合适的字体，使用OpenCV的英文显示
            cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            return img
    
    def display_frames(self):
        """显示两个相机的彩色图像"""
        try:
            # 创建命名窗口，确保窗口会显示
            cv2.namedWindow('Camera 1', cv2.WINDOW_AUTOSIZE)
            if self.serial_2:
                cv2.namedWindow('Camera 2', cv2.WINDOW_AUTOSIZE)
            
            while True:
                # 获取帧
                color_1, color_2 = self.get_frames()
                
                # 显示全局相机图像
                if color_1 is not None:
                    try:
                        # 添加标签到图像
                        color_1_display = color_1.copy()
                        color_1_display = self.put_chinese_text(color_1_display, "全局相机", (10, 30), (0, 255, 0))
                        
                        # 显示图像
                        cv2.imshow('Camera 1', color_1_display)
                    except Exception as e:
                        print(f"显示全局相机图像时出错: {e}")
                else:
                    # 如果没有图像，显示黑屏带错误信息
                    blank_image = np.zeros((480, 640, 3), np.uint8)
                    blank_image = self.put_chinese_text(blank_image, "无法获取全局相机图像", (50, 240), (0, 0, 255))
                    cv2.imshow('Camera 1', blank_image)
                
                # 显示腕部相机图像
                if color_2 is not None:
                    try:
                        # 添加标签到图像
                        color_2_display = color_2.copy()
                        color_2_display = self.put_chinese_text(color_2_display, "腕部相机", (10, 30), (0, 255, 0))
                        
                        # 显示图像
                        cv2.imshow('Camera 2', color_2_display)
                    except Exception as e:
                        print(f"显示腕部相机图像时出错: {e}")
                elif self.serial_2:
                    # 如果没有图像，显示黑屏带错误信息
                    blank_image = np.zeros((480, 640, 3), np.uint8)
                    blank_image = self.put_chinese_text(blank_image, "无法获取腕部相机图像", (50, 240), (0, 0, 255))
                    cv2.imshow('Camera 2', blank_image)
                
                # 刷新窗口，确保显示
                cv2.waitKey(1)
                
                # 检查是否按下'q'键退出
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("用户按下'q'键，退出程序")
                    break
                
                # 短暂暂停，减少CPU使用
                time.sleep(0.01)
                
        except Exception as e:
            print(f"显示帧时出错: {e}")
        
        finally:
            print("关闭所有窗口")
            cv2.destroyAllWindows()
            # 多次调用，确保窗口关闭
            for i in range(5):
                cv2.waitKey(1)
    
    def run(self):
        """运行相机查看器"""
        if self.start():
            try:
                print("开始显示相机图像。按'q'键退出。")
                self.display_frames()
            finally:
                self.stop()
        else:
            print("启动相机失败")


if __name__ == "__main__":
    viewer = CameraViewer()
    viewer.run()