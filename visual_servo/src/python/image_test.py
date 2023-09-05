import sys
sys.path.append('c:\\Users\\chen\\Desktop\\line_tracker\\visual_servo\\')
from utils.image_features import *
from utils.camera_related import pixel2mm
from utils.extract_features import find_red_circles
import airsim
import cv2
import numpy as np
import time

if __name__ == "__main__":

    # 定义相关参数,列向量
    q1_ = np.array([0, 0, 0, 0]).reshape(4, 1)  # 特征的期望值
    q1 = np.array([0, 0, 0, 0]).reshape(4, 1)
    a_ = 1  # 这个是图像特征的一个期望值
    z_ = 4  # 期望高度，可能不应该是m
    lambda_ = 3.2
    c1 = 1
    c2 = 1
    c3 = 1
    faid = 0
    fai = 0
    v = np.array([0, 0, 0]).reshape(3, 1)  # 虚坐标系下的速度
    pre_time = time.time()
    while True:
        # 计算相邻图像的获取时间间隔
        now_time = time.time()
        print("time_______________")
        print(now_time-pre_time)
        pre_time = now_time
        # 获取rgb图像，下视的 大小为640*480
        img_rgb=cv2.imread('c:\\Users\\chen\\Desktop\\line_tracker\\visual_servo\\img\\red_balls.png')
        # 展示图像
        cv2.imshow("img_rgb", img_rgb)
        # 绘制期望点
        cv2.waitKey(1)
        # 将图像送入find_blue_block处理
        p_cam = find_red_circles(img_rgb)  # 这里面会进行绘制
        if p_cam is None:
            client.hoverAsync().join()
            continue
        print("p_cam_______________")
        print(p_cam)
        # 将p_cam通过pixel2mm转为mm
        p_mm = np.zeros(8).reshape(8, 1)
        for i in range(4):
            p_mm[2*i:2*(i+1)] = pixel2mm(3.2, 1, 1, p_cam[2*i], p_cam[2*i+1])
        # 计算期望点的特征
        q1 = cal_image_features(
            p_cam[0:8:2], p_cam[1:8:2], a_, lambda_)
        # 计算uf(期望速度（虚）与期望角速度)
        uf = cal_uf(uf, 0.1, c1, c2, z_, faid, fai, q1, q1_, v,)

