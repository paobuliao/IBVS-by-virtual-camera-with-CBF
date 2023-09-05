import cv2 
import numpy as np
import time

def find_red_circles(image):
    # 转换图像到HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义红色的HSV范围（考虑不同光照情况可能需要微调）
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    # 创建一个红色掩码，只保留红色区域
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    # 使用霍夫圆变换检测红色通道中的所有圆
    circles = cv2.HoughCircles(
        red_mask, 
        cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=7, param2=7, minRadius=0, maxRadius=20
    )
    return_array=[]
    if circles is not None:
        circles = np.uint16(np.around(circles))
        
        # 找到并保留最大的四个圆
        largest_circles = sorted(circles[0, :], key=lambda x: x[2], reverse=True)[:4]
        
        for circle in largest_circles:
            center = (circle[0], circle[1])  # 圆心坐标
            radius = circle[2]  # 半径
            return_array.append([center[0],center[1],radius])#半径之后再考虑，可能也是一种好用的图像特征
            # 在原始图像上绘制圆
            cv2.circle(image, center, radius, (0, 255, 0), 2)
            
            # 在圆心位置绘制一个小的红点
            cv2.circle(image, center, 2, (0, 0, 255), 3)
        # 显示图像
        cv2.imshow('LRed Circles', image)
        cv2.waitKey(1)
    #以np.array格式返回
    return np.array(return_array)


def find_red_block(image):
    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义红色的HSV范围
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # 根据红色的HSV范围创建蒙版
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # 执行形态学操作以去除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 如果找到轮廓
    if len(contours) > 0:
        # 找到最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)

        # 计算轮廓的边界框
        x, y, w, h = cv2.boundingRect(max_contour)

        # 计算中心位置
        center_x = x + w / 2
        center_y = y + h / 2
        #绘制轮廓
        #绘制返回值点
        cv2.circle(image, (int(center_x-w/2), int(center_y-h/2)), 5, (255, 0, 0), -1)
        cv2.circle(image, (int(center_x+w/2), int(center_y+h/2)), 5, (255, 0, 0), -1)
        cv2.circle(image, (int(center_x-w/2), int(center_y+h/2)), 5, (255, 0, 0), -1)
        cv2.circle(image, (int(center_x+w/2), int(center_y-h/2)), 5, (255, 0, 0), -1)
        #展示图像
        cv2.imshow("image", image)
        cv2.waitKey(1)
        # 以np.array格式返回
        return np.array([ [int(center_x-w/4), int(center_y-h/2)], 
                          [int(center_x+w/4), int(center_y-h/2)], 
                          [int(center_x-w/4),int(center_y+h/2)], 
                          [int(center_x+w/4),int(center_y+h/2)]])
        # return np.array([center_x, center_y,w,h])
    
    # 如果未找到蓝色色块，则返回None
    return None

def find_red_block_2(image):
    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义红色的HSV范围
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # 根据红色的HSV范围创建蒙版
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # 执行形态学操作以去除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 如果找到轮廓
    if len(contours) > 0:
        # 找到最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)

        # 计算轮廓的边界框
        x, y, w, h = cv2.boundingRect(max_contour)

        # 计算中心位置
        center_x = x + w / 2
        center_y = y + h / 2
        #绘制轮廓
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(img_rgb, (exp[0], exp[1]), 5, (0, 255, 0), -1)
        cv2.circle(img_rgb, (exp[2], exp[3]), 5, (0, 255, 0), -1)
        cv2.circle(img_rgb, (exp[4], exp[5]), 5, (0, 255, 0), -1)
        cv2.circle(img_rgb, (exp[6], exp[7]), 5, (0, 255, 0), -1)
        #绘制返回值点
        cv2.circle(img_rgb, (int(center_x-w/2), int(center_y-h/2)), 5, (255, 0, 0), -1)
        cv2.circle(img_rgb, (int(center_x+w/2), int(center_y+h/2)), 5, (255, 0, 0), -1)
        cv2.circle(img_rgb, (int(center_x-w/2), int(center_y+h/2)), 5, (255, 0, 0), -1)
        cv2.circle(img_rgb, (int(center_x+w/2), int(center_y-h/2)), 5, (255, 0, 0), -1)
        #展示图像
        cv2.imshow("img_rgb", image)
        cv2.waitKey(1)
        # 以np.array格式返回
        return np.array([center_x, center_y,w,h])
    
    # 如果未找到蓝色色块，则返回None
    return None