import numpy as np


def skew(v):
    result = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])
    return result

import numpy as np

def cal_image_average(u, v):
    """
    Calculate the image average.

    Parameters:
        u: List of horizontal pixel coordinates
        v: List of vertical pixel coordinates (should be the same size as u)

    Returns:
        List containing [ug, vg] (average horizontal and vertical coordinates)
    """
    ug = sum(u) / len(u)
    vg = sum(v) / len(v)
    return [ug, vg]


def cal_image_moment(u, v, i, j):
    """
    Calculate image moment.

    Parameters:
        u: List of horizontal pixel coordinates
        v: List of vertical pixel coordinates (should be the same size as u)
        i: Order of moment
        j: Order of moment

    Returns:
        Image moment value
    """
    ave_result = cal_image_average(u, v)
    ug, vg = ave_result[0], ave_result[1]

    m = sum([(ui - ug) ** i * (vi - vg) ** j for ui, vi in zip(u, v)])
    return m


def cal_image_features(u, v, a_, lambda_):
    '''
    params:
    u,v:像素坐标
    a_:a的期望值
    lambda_:相机焦距
    '''
    ave_result = cal_image_average(u, v)
    ug, vg = ave_result[0][0], ave_result[1][0]

    m11 = cal_image_moment(u, v, 1, 1)[0]
    m20 = cal_image_moment(u, v, 2, 0)[0]
    m02 = cal_image_moment(u, v, 0, 2)[0]

    a = m20 + m02
    qz = np.sqrt(a_ / a)
    qx = qz * ug / lambda_
    qy = qz * vg / lambda_
    if m20==m02:
        q4=np.pi/4
    else:
        q4 = 0.5 * np.arctan(2 * m11 / (m20 - m02))

    return np.array([qx, qy, qz, q4]).reshape(4,1)


def cal_delta(c1, c2, q1, q1_, z_, uf, q1d):
    H = np.array([[-1 / z_, 0, 0, q1[1, 0]],
                  [0, -1 / z_, 0, -q1[0, 0]],
                  [0, 0, -1 / z_, 0],
                  [0, 0, 0, -1]])
    delta = c1*(q1-q1_)+c2*(H@uf-q1d)
    return delta


def cal_q1d(q1, v, psid, z_):
    '''
    该函数用于计算q1d
    需要获取加速度faid
    与虚相平面在虚坐标系的速度
    '''
    e3 = np.array([0, 0, 1])
    q1_temp = q1[0:3]
    q1d = -np.dot(skew(psid * e3), q1_temp) - (1 / z_) * v
    q4d = -psid
    return np.array([[q1d[0,0]], [q1d[1,0]], [q1d[2,0]], [q4d]])


def cal_ufd(c1, c2, c3, z_,  q1, q1d, uf, delta):
    '''
    这里忽视了q的期望值与时间的关系,也就是q是不变的
    '''
    H = np.array([[-1 / z_, 0, 0, q1[1, 0]],
                  [0, -1 / z_, 0, -q1[0, 0]],
                  [0, 0, -1 / z_, 0],
                  [0, 0, 0, -1]])
    Hd = np.array([[0, 0, 0, q1d[1, 0]],
                   [0, 0, 0, -q1d[0, 0]],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]])
    ufd = np.dot(np.linalg.inv(c2 * H), -(c1 * H + c2 * Hd) @ uf - c3*delta)
    return ufd


def cal_uf(uf, dt, c1, c2, c3, z_, psid, q1, q1_, v):
    '''
    uf是虚相坐标系的速度与偏航角速度
    该函数用四阶龙格库塔法计算uf
    '''
    # 预备材料
    q1d = cal_q1d(q1, v, psid, z_)
    delta = cal_delta(c1, c2, q1, q1_, z_, uf, q1d)

    uf1 = uf
    uf1d = cal_ufd(c1, c2, c3, z_, q1, q1d, uf, delta)
    uf2 = uf1 + (dt / 2) * uf1d
    uf2d = cal_ufd(c1, c2, c3, z_,  q1, q1d, uf2, delta)
    uf3 = uf1 + (dt / 2) * uf2d
    uf3d = cal_ufd(c1, c2, c3, z_, q1, q1d, uf3, delta)
    uf4 = uf1 + dt * uf3d
    uf4d = cal_ufd(c1, c2, c3, z_, q1, q1d, uf4, delta)
    uf = uf1 + dt * (uf1d + 2 * uf2d + 2 * uf3d + uf4d)/6
    ufd=(uf1d + 2 * uf2d + 2 * uf3d + uf4d)/6
    return uf,ufd


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


if __name__ == "__main__":
    p=np.array([[388,190,10]])
    p_mm = np.zeros(8).reshape(8, 1)
    for i in range(len(p)):
        p_mm[2*i], p_mm[2*i +
                        1] = pixel2mm(320, 320, 3.2, p[i, 0], p[i, 1])
    q1 = cal_image_features(
            p_mm[0:2*len(p):2], p_mm[1:2*len(p):2], a_, lambda_)