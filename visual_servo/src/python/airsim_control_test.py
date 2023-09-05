import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
import airsim
import sys
sys.path.append('c:\\Users\\chen\\Desktop\\line_tracker\\visual_servo\\')
from utils.extract_features import find_red_circles
from utils.camera_related import pixel2mm
from utils.image_features import *
from utils.controller import Airsim_quadrotor
from utils.attitude_func import euler2rotmat  #机体到世界

if __name__ == "__main__":
    my_quadrotor = Airsim_quadrotor()
    # 创建airsim连接
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    # 让飞机起飞并升高十米
    client.takeoffAsync().join()
    client.moveToPositionAsync(1, 0, -5, 5).join()
    # to do
    # 初始化变量
    # 这一个仿真中无人机的参数假设能准确获取
    uav_state = client.simGetGroundTruthKinematics(vehicle_name='UAV1')
    #记录它的姿态角，角速度，速度
    ang = np.array([airsim.to_eularian_angles(uav_state.orientation)]).reshape(3,1)
    ang_vel=np.array([uav_state.angular_velocity.x_val,
                        uav_state.angular_velocity.y_val,uav_state.angular_velocity.z_val])
    v_inertial = np.array([uav_state.linear_velocity.x_val,
                          uav_state.linear_velocity.y_val, uav_state.linear_velocity.x_val]).reshape(3,1)
    ang_vel_des = np.array([0.0, 0.0, 1]).reshape(3, 1)
    angular_vel=[]
    while True:
        # 更新无人机的状态
        uav_state = client.simGetGroundTruthKinematics(vehicle_name='UAV1')
        ang = np.array([airsim.to_eularian_angles(uav_state.orientation)]).reshape(3,1)
        ang_vel=np.array([uav_state.angular_velocity.x_val,
                        uav_state.angular_velocity.y_val,uav_state.angular_velocity.z_val]).reshape(3,1)
        v_inertial = np.array([uav_state.linear_velocity.x_val,
                            uav_state.linear_velocity.y_val, uav_state.linear_velocity.x_val]).reshape(3,1)
        angular_vel.append(ang_vel)
        print('-----------------ang_vel----------------')
        print(ang_vel)
        # 由姿态角获取旋转矩阵
        R = euler2rotmat(ang) #机体到世界
        e_3=np.array([0,0,-1]).reshape(3,1)
        transformed_e_3=np.dot(R,e_3)
        #计算期望总升力
        f=-my_quadrotor.m*my_quadrotor.g/transformed_e_3[2,0]+0.3
        print('-----------------f----------------')
        print(f)
        M=my_quadrotor.P_Angular_Vel_Controller(ang_vel,ang_vel_des)
        u1,u2,u3,u4=my_quadrotor.fM2u(f,M)
        print('-----------------u----------------')
        print(u1,u2,u3,u4)
        client.moveByMotorPWMsAsync(u1, u2, u3, u4, 0.1, vehicle_name='UAV1')
        time.sleep(0.1)
    #绘制角速度
    angular_vel=np.array(angular_vel)
    plt.plot(angular_vel[:,0])
    plt.plot(angular_vel[:,1])
    plt.plot(angular_vel[:,2])
    plt.show()
