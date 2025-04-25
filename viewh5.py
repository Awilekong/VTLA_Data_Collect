import h5py
import cv2
import numpy as np

# 打开 .h5 文件
with h5py.File('collected_data/robot_data_20250424_160948.h5', 'r') as f:
    images = f['observation.images.image'][:] # shape: (T, H, W, C)
    images_wrist = f['observation.images.wrist_image'][:] # shape: (T, H, W, C)

    # 播放每一帧
for i, img in enumerate(images_wrist):
    # 如果是 RGB，OpenCV 需要转换为 BGR 显示
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    cv2.imshow('Video from .h5', img_bgr)
    key = cv2.waitKey(100) # 每帧等待100ms（10fps）

    if key == 27: # 按下 ESC 键退出
        break

cv2.destroyAllWindows()