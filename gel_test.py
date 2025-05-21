#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 修改自 tracking.py

import copy
from gelmini import find_marker
import numpy as np
import cv2
import time
from gelmini import marker_detection
import sys
from gelmini import setting
import os
from gelmini.gsdevice import Camera, resize_crop_mini

def main():
    # 初始化参数
    imgw = 320
    imgh = 240

    # 设置单个窗口用于显示拼接图像
    cv2.namedWindow('GelSight双传感器形变场', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('GelSight双传感器形变场', 320*6, 240*3)  # 左右拼接，所以宽度翻倍

    # 使用gsdevice.py方法连接两个GelSight传感器
    print("正在连接第一个GelSight Mini传感器...")
    gel1 = Camera()
    gel1.connect()
    try:
        gel1.connect()
    except Exception as e:
        print(f"错误: 无法连接第一个GelSight传感器: {e}")
        if hasattr(gel1, 'cam') and gel1.cam is not None:
            gel1.cam.release()
        cv2.destroyAllWindows()
        sys.exit(1)
    
    print("正在连接第二个GelSight Mini传感器...")
    gel2 = Camera()
    try:
        gel2.connect()
    except Exception as e:
        print(f"错误: 无法连接第二个GelSight传感器: {e}")
        # 释放资源
        if hasattr(gel1, 'cam') and gel1.cam is not None:
            gel1.cam.release()
        cv2.destroyAllWindows()
        sys.exit(1)
    
    # 初始化setting
    setting.init()
    
    # 获取初始帧并进行标记点检测
    print("正在初始化标记点检测...")
    # 第一个传感器
    for i in range(50):  # 清除黑帧
        frame1 = gel1.get_image()
        if i == 48:
            mask1 = marker_detection.find_marker(frame1)
            mc1 = marker_detection.marker_center(mask1, frame1)
    
    # 第二个传感器
    for i in range(50):  # 清除黑帧
        frame2 = gel2.get_image()
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
    N_ = setting.N_
    M_ = setting.M_
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
    matcher1 = find_marker.Matching(N_, M_, fps_, x0_1, y0_1, dx_1, dy_1)
    matcher2 = find_marker.Matching(N_, M_, fps_, x0_2, y0_2, dx_2, dy_2)
    
    # 存储初始帧用于比较
    frame1_0 = frame1.copy()
    frame1_0 = cv2.GaussianBlur(frame1_0, (int(63), int(63)), 0)
    
    frame2_0 = frame2.copy()
    frame2_0 = cv2.GaussianBlur(frame2_0, (int(63), int(63)), 0)
    
    # 主循环
    print("开始实时显示双传感器形变场...")
    try:
        while True:
            # 获取图像
            frame1 = gel1.get_image()
            frame2 = gel2.get_image()
            
            if frame1 is None or frame2 is None:
                print("无法获取图像，检查传感器连接")
                break
                
            # 复制原始图像用于显示
            display1 = copy.deepcopy(frame1)
            display2 = copy.deepcopy(frame2)
            
            # 检测标记点
            mask1 = marker_detection.find_marker(frame1)
            mc1 = marker_detection.marker_center(mask1, frame1)
            
            mask2 = marker_detection.find_marker(frame2)
            mc2 = marker_detection.marker_center(mask2, frame2)
            
            # 匹配和跟踪标记点
            matcher1.init(mc1)
            matcher1.run()
            flow1 = matcher1.get_flow()
            
            matcher2.init(mc2)
            matcher2.run()
            flow2 = matcher2.get_flow()
            
            # 绘制形变场
            marker_detection.draw_flow(display1, flow1)
            marker_detection.draw_flow(display2, flow2)
            
            # 调整图像大小以便更好地显示
            display1 = cv2.resize(display1, (display1.shape[1]*3, display1.shape[0]*3))
            display2 = cv2.resize(display2, (display2.shape[1]*3, display2.shape[0]*3))
            
            # 添加标签以区分两个传感器
            cv2.putText(display1, "传感器 1", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                      1, (0, 255, 0), 2)
            cv2.putText(display2, "传感器 2", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                      1, (0, 255, 0), 2)
            
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
            cv2.imshow('GelSight双传感器形变场', combined_display)
            
            # 按q退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            # 控制刷新率
            time.sleep(0.05)  # 约20fps
            
    except KeyboardInterrupt:
        print('\n已中断!')
    finally:
        # 释放资源
        print("清理资源...")
        if hasattr(gel1, 'cam') and gel1.cam is not None:
            gel1.cam.release()
        if hasattr(gel2, 'cam') and gel2.cam is not None:
            gel2.cam.release()
        cv2.destroyAllWindows()
        print("程序已安全退出")

if __name__ == "__main__":
    main()