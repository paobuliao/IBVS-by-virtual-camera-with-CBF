import numpy as np
from .image_features import skew
from .quaternion_euler_utility import *


def cal_f(uf, ufd, psid, v, k1):  # 这个psid不知道是实际值还是测量值
    v_d = ufd[0:3]
    v_ = uf[0:3]
    e3 = np.array([0, 0, 1])
    f = v_d+skew(psid*e3)@v-k1*(v-v_)
    return f


def cal_desired_three_elements(uf, ufd, psid, v, k1, ang, m):
    f = cal_f(uf, ufd, psid, v, k1)
    e3 = np.array([0, 0, 1]).reshape(3, 1)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(ang[0,0]), -np.sin(ang[0,0])],
        [0, np.sin(ang[0,0]), np.cos(ang[0,0])]
    ])
    Ry = np.array([
        [np.cos(ang[1,0]), 0, np.sin(ang[1,0])],
        [0, 1, 0],
        [-np.sin(ang[1,0]), 0, np.cos(ang[1,0])]
    ])
    Rz = np.array([
        [np.cos(ang[2,0]), -np.sin(ang[2,0]), 0],
        [np.sin(ang[2,0]), np.cos(ang[2,0]), 0],
        [0, 0, 1]
    ])
    Ryx = np.dot(Ry, Rx)
    U1 = np.dot(e3.T, np.dot(Ryx.T, m*9.81*e3-m*f))# 可能因为计算f用的期望值，而U1用的实际值，所以不行
    print('---------------------f---------------------')
    print(f)
    fai = np.arcsin(f[1, 0]/U1)
    theta = np.arcsin(-f[0, 0]/(U1*np.cos(fai)))
    return U1, fai, theta


def integrate_psi(psi, psi_dot, dt):
    psi = psi+psi_dot*dt
    return psi


class Airsim_quadrotor():
    def __init__(self):
        self.J = np.array(
            [[0.006721, 0, 0], [0, 0.00804, 0], [0, 0, 0.014279]])
        self.Ixx = 0.006721
        self.Iyy = 0.00804
        self.Izz = 0.014279
        # Mass and Gravity
        self.m, self.g = 1, 9.81
        self.d = 0.2275

    def PD_Att_Controller(self, ang_now, ang_vel_now, ang_des):
        phi = float(ang_now[0])
        theta = float(ang_now[1])
        psi = float(ang_now[2])

        # PID gains unlimited
        Kp = np.array([[2, 0, 0],
                       [0, 2, 0],
                       [0, 0, 0.01]])*3
        Kd = np.array([[0.1, 0, 0],
                       [0, 0.1, 0],
                       [0, 0, .03]])*1

        angle_error = ang_des - ang_now
        # print('Erro angulo:', angle_error.T)
        ang_vel_error = np.zeros((3, 1)) - ang_vel_now
        # Compute Optimal Control Law

        T = np.array([[1/self.Ixx, np.sin(phi)*np.tan(theta)/self.Iyy, np.cos(phi)*np.tan(theta)/self.Izz],
                      [0, np.cos(phi)/self.Iyy, -np.sin(phi)/self.Izz],
                      [0, np.sin(phi)*np.cos(theta)/self.Iyy, np.cos(phi)*np.cos(theta)/self.Izz]])

        u = np.linalg.inv(T)@(Kp@angle_error + Kd@ang_vel_error)  # 求出期望的角速度
        tau = self.P_Angular_Vel_Controller(ang_vel_now, u)  # 用我的P期望角速度控制器求出力矩

        return tau

    def P_Angular_Vel_Controller(self, ang_vel_now, ang_vel_des):
        # PID gains unlimited
        Kp = np.array([[1, 0, 0],
                       [0, 1, 0],
                       [0, 0, 1]])*1
        Kd = np.array([[0.2, 0, 0],
                       [0, 0.2, 0],
                       [0, 0, 0.2]])*1

        ang_vel_error = ang_vel_des - ang_vel_now
        # Compute Optimal Control Law
        u = Kp@ang_vel_error
        tau = self.J@u+np.cross(ang_vel_now.transpose(),
                                (self.J@ang_vel_now).T).T
        return tau.reshape(3, 1)

    # 我现在有U, faid, thetad, psid
    # 转化为各旋翼呃转速
        # 力和力矩到电机控制的转换
    def fM2u(self, f, M):
        mat = np.array([[4.179446268, 4.179446268, 4.179446268, 4.179446268],
                        [-0.6723341164784, 0.6723341164784,
                            0.6723341164784, -0.6723341164784],
                        [0.6723341164784, -0.6723341164784,
                            0.6723341164784, -0.6723341164784],
                        [0.055562, 0.055562, -0.055562, -0.055562]])
        fM = np.vstack([f, M])
        u = np.dot(np.linalg.inv(mat), fM)
        u1 = u[0, 0]
        u2 = u[1, 0]
        u3 = u[2, 0]
        u4 = u[3, 0]
        return u1, u2, u3, u4
