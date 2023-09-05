import sys
sys.path.append('c:\\Users\\chen\\Desktop\\line_tracker\\visual_servo\\')
from utils.controller import *
from utils.image_features import *
from utils.camera_related import pixel2mm,get_virtual_coor
from utils.extract_features import find_red_circles,find_red_block
from utils.attitude_func import euler2rotmat
from config.config import *
import time
import numpy as np
import cv2
import airsim
#创建一个线程实现图像处理
import threading

class image_Thread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        #这个函数会改变一些全局变量
        # global p_cam
        global client1
        global ang
        global q1
        while True:
            responses = client1.simGetImages(
                    [airsim.ImageRequest("3", airsim.ImageType.Scene, False, False)])
            response = responses[0]
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            # 展示图像
            cv2.imshow("img_rgb", img_rgb)
            # 绘制期望点
            cv2.waitKey(1)
            # 将图像送入find_blue_block处理
            p_cam = find_red_circles(img_rgb)  # 这里面会进行绘制
            if p_cam is None or len(p_cam) <2:
                client.hoverAsync().join()
                continue
            # print("-------------------p_cam-------------------")
            # print(p_cam)
            for i in range(len(p_cam)):
                p_cam[i, 0], p_cam[i, 1] = get_virtual_coor(ang, p_cam[i, 0], p_cam[i, 1], f=320, frombody=True)
            # print("--------------transformed--------------------")
            # print(p_cam)
            # 将p_cam通过pixel2mm转为m  不知道用m还是mm
            p_mm = np.zeros(8).reshape(8, 1)
            for i in range(len(p_cam)):
                p_mm[2*i], p_mm[2*i +
                                1] = pixel2mm(320, 320, 3.2, p_cam[i, 0], p_cam[i, 1])
            # 计算期望点的特征
            q1 = cal_image_features(
                p_mm[0:2*len(p_cam):2], p_mm[1:2*len(p_cam):2], a_, lambda_)
            # print('---------------------q1--------------------')
            # print(q1)
            # time.sleep(0.1)

def trajectory(time_delta):
    if time_delta<5:
        qx=a*time_delta**3+b*time_delta**2+d
        qy=0.2
    elif time_delta<10:
        qy=a*(time_delta-5)**3+b*(time_delta-5)**2+d
        qx=-0.2
    elif time_delta<15:
        qx=-a*(time_delta-10)**3-b*(time_delta-10)**2-d
        qy=-0.2
    else:
        qy=-a*(time_delta-15)**3-b*(time_delta-15)**2-d
        qx=0.2
    return qx,qy

if __name__ == "__main__":
    my_quadrotor = Airsim_quadrotor()
    # 创建airsim连接
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # 这个连接用于图像处理
    client1= airsim.MultirotorClient()
    client.confirmConnection()

    client.enableApiControl(True)
    client.armDisarm(True)
    # 让飞机起飞并升高十米
    client.takeoffAsync().join()
    client.moveToPositionAsync(-1, 0, -10, 5).join()
    # to do

    # 这一个仿真中无人机的参数假设能准确获取
    uav_state = client.simGetGroundTruthKinematics(vehicle_name='')
    ang = np.array([airsim.to_eularian_angles(
        uav_state.orientation)]).reshape(3, 1)
    ang_vel = np.array([uav_state.angular_velocity.x_val, uav_state.angular_velocity.y_val,
                       uav_state.angular_velocity.z_val]).reshape(3, 1)
    v_inertial = np.array([uav_state.linear_velocity.x_val,
                          uav_state.linear_velocity.y_val, uav_state.linear_velocity.z_val]).reshape(3, 1)
    R_b2w=euler2rotmat(ang)
    v=R_b2w.T@v_inertial
    pre_time = time.time()
    ori_time = pre_time
    # 开启图像处理线程,这个线程会改变一些全局变量
    img_process_thread=image_Thread()
    img_process_thread.start()
    while True:
        # 计算相邻图像的获取时间间隔
        now_time = time.time()
        time_delta=now_time-ori_time
        #desired features
        qx,qy=trajectory(time_delta)
        # q1_ = np.array([qx, qy,1 , 0]).reshape(4, 1)
        dt = now_time-pre_time
        if(dt>0.25):
            uf = np.array([0, 0, 0, 0]).reshape(4, 1)
        print("-----------------------dt----------------------")
        print(dt)
        pre_time = now_time
        # 获取rgb图像，下视的 大小为640*480

        # 计算uf(期望速度（虚）与期望角速度)
        uf, ufd = cal_uf(uf, dt, c1, c2, c3, z_, psi_d, q1, q1_, v)# 如果没有收到图像，让它按之前的数据一直更新怎么样
        psi_d=ufd[3,0]
        print('------------------------uf-----------------')
        print(uf)
        psi_ = integrate_psi(psi_, uf[3], dt)
        ang_vel_world=R_b2w.T@ang_vel
        psid=ang_vel_world[2,0]# 不确定airsim给的是机体的还是世界的，有没有必要转化
        U1, fai, theta = cal_desired_three_elements(
            uf, ufd, psid, v, k1, ang, my_quadrotor.m)#我把这个换成真实的角速度试试？
        ang_des = np.array([fai, theta, psi_]).reshape(3, 1)
        print('---------------------U1------------------ ')
        print(U1)
        # PD控制器计算期望角速度，P控制器计算期望力矩
        print('--------------------ang_des-------------------')
        print(ang_des)
        print('--------------------ang-------------------')
        print(ang)
        M = my_quadrotor.PD_Att_Controller(ang, ang_vel, ang_des)
        u1, u2, u3, u4 = my_quadrotor.fM2u(U1, M)
        # print('-----------------u----------------')
        # print(u1, u2, u3, u4)
        client.moveByMotorPWMsAsync(u1, u2, u3, u4, 0.1, vehicle_name='UAV1')
        # time.sleep(0.5)

        # 更新无人机的状态
        uav_state = client.simGetGroundTruthKinematics(vehicle_name='')
        ang = np.array([airsim.to_eularian_angles(
            uav_state.orientation)]).reshape(3, 1)
        ang_vel = np.array([uav_state.angular_velocity.x_val, uav_state.angular_velocity.y_val,
                           uav_state.angular_velocity.z_val]).reshape(3, 1)
        v_inertial = np.array([uav_state.linear_velocity.x_val,
                               uav_state.linear_velocity.y_val, uav_state.linear_velocity.z_val]).reshape(3, 1)
        R_b2w=euler2rotmat(ang)
        v=R_b2w.T@v_inertial
        print('----------------------v--------------------')
        print(v)