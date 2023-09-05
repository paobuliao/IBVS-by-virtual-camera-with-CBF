import numpy as np


def pixel2mm(fx, fy, lamb, u, v):
    '''
    params:
    fx,fy:相机内参
    lamb:相机焦距,这个可能只要比例关系对了就没事? 那我假设它为3.2mm,像素长度为1mm,图像大小为640*480
    u,v:像素坐标
    '''
    x = (u-320)*lamb*10**(-5)
    y = (v-240)*lamb*10**(-5)
    return x, y

# this function is used to calculate the coordinate in virtual image plane


def get_virtual_coor(ang, uo,vo, f=320, frombody=True):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(ang[0,0]), -np.sin(ang[0,0])],
                   [0, np.sin(ang[0,0]), np.cos(ang[0,0])]])

    Ry = np.array([[np.cos(ang[1,0]), 0, np.sin(ang[1,0])],
                   [0, 1, 0],
                   [-np.sin(ang[1,0]), 0, np.cos(ang[1,0])]])

    Rz = np.array([[np.cos(ang[2,0]), -np.sin(ang[2,0]), 0],
                   [np.sin(ang[2,0]), np.cos(ang[2,0]), 0],
                   [0, 0, 1]])

    R = Rz.dot(Ry).dot(Rx)

    if not frombody:
        R = R.T

    # print(R)

    uf = (R[0, 0] * uo + R[0, 1] * vo + f * R[0, 2]) / \
        (1/f * R[2, 0] * uo + 1/f * R[2, 1] * vo + R[2, 2])
    vf = (R[1, 0] * uo + R[1, 1] * vo + f * R[1, 2]) / \
        (1/f * R[2, 0] * uo + 1/f * R[2, 1] * vo + R[2, 2])

    return uf,vf
