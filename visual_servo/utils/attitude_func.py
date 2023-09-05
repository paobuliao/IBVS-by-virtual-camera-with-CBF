import numpy as np
import math


def quaternConj(q):
    q = np.mat([q[0, 0], -q[0, 1], -q[0, 2], -q[0, 3]])
    return q


def quaternProd(a, b):
    ab = np.zeros([1, 4])
    ab[0, 0] = a[0, 0] * b[0, 0] - a[0, 1] * b[0, 1] - a[0, 2] * b[0, 2] - a[0, 3] * b[0, 3]
    ab[0, 1] = a[0, 0] * b[0, 1] + a[0, 1] * b[0, 0] + a[0, 2] * b[0, 3] - a[0, 3] * b[0, 2]
    ab[0, 2] = a[0, 0] * b[0, 2] - a[0, 1] * b[0, 3] + a[0, 2] * b[0, 0] + a[0, 3] * b[0, 1]
    ab[0, 3] = a[0, 0] * b[0, 3] + a[0, 1] * b[0, 2] - a[0, 2] * b[0, 1] + a[0, 3] * b[0, 0]
    return ab

def quat2rotmat(quatern):
    #这个应该是正确的
    q0 = quatern[0, 0]
    q1 = quatern[0, 1]
    q2 = quatern[0, 2]
    q3 = quatern[0, 3]
    r11 = q0**2+q1**2-q2**2-q3**2
    r12=2*(q1*q2-q0*q3)
    r13=2*(q1*q3+q0*q2)
    r21=2*(q1*q2+q0*q3)
    r22=q0**2-q1**2+q2**2-q3**2
    r23=2*(q2*q3-q0*q1)
    r31=2*(q1*q3-q0*q2)
    r32=2*(q2*q3+q0*q1)
    r33=q0**2-q1**2-q2**2+q3**2
    rotation_mat = np.array([[r11, r12, r13],
                           [r21, r22, r23],
                           [r31, r32, r33]])
    return rotation_mat


def quat2euler(quatern):
    #这个很可能有问题
    q0 = quatern[0, 0]
    q1 = quatern[0, 1]
    q2 = quatern[0, 2]
    q3 = quatern[0, 3]
    fai = math.atan2(2 * (q2 * q3 + q0 * q1), q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2)
    # theta=-math.asin(2*(q1*q3-q0*q2))
    # theta = math.asin(2 * (q0 * q2 - q3 * q1))
    theta=math.atan2(2*(q0*q2-q1*q3),(4*(q0*q1+q2*q3)**2+(1-2*(q1**2+q2**2))**2))
    psai = math.atan2(2 * (q1 * q2 + q0 * q3), q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2)
    return np.mat([fai, theta, psai])


def euler2rotmat(euler):
    ''' 这个函数还没测试过，还不知道能不能用
    相机到世界
    :param euler:
    :return:
    '''
    roll = euler[0, 0]
    pitch = euler[1, 0]
    yaw = euler[2, 0]
    r11 = math.cos(yaw) * math.cos(pitch)
    r12 = math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll)
    r13 = math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)
    r21 = math.sin(yaw) * math.cos(pitch)
    r22 = math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll)
    r23 = math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)
    r31 = -math.sin(pitch)
    r32 = math.cos(pitch) * math.sin(roll)
    r33 = math.cos(pitch) * math.cos(roll)
    rotation_mat = np.mat([[r11, r12, r13],
                           [r21, r22, r23],
                           [r31, r32, r33]])
    return rotation_mat

def rotmat2euler(R):
    fai=math.atan2(R[2,1],R[2,2])
    theta=math.atan2(-R[2,0],math.sqrt(R[2,1]**2+R[2,2]**2))
    psai=math.atan2(R[1,0],R[0,0])
    return np.mat([fai,theta,psai])

def w_world2w_body(euler):
    roll = euler[0, 0]
    pitch = euler[0, 1]
    yaw = euler[0, 2]
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    rotation_mat = np.mat([[1, 0, -sp],
                           [0, cr, sr * cp],
                           [0, -sr, cr * cp]])
    return rotation_mat


def euler2quat(euler):
    x = euler[0, 0]
    y = euler[0, 1]
    z = euler[0, 2]
    q0 = math.cos(x / 2) * math.cos(y / 2) * math.cos(z / 2) + math.sin(x / 2) * math.sin(y / 2) * math.sin(z / 2)
    q1 = math.sin(x / 2) * math.cos(y / 2) * math.cos(z / 2) - math.cos(x / 2) * math.sin(y / 2) * math.sin(z / 2)
    q2 = math.cos(x / 2) * math.sin(y / 2) * math.cos(z / 2) + math.sin(x / 2) * math.cos(y / 2) * math.sin(z / 2)
    q3 = math.cos(x / 2) * math.cos(y / 2) * math.sin(z / 2) - math.sin(x / 2) * math.sin(y / 2) * math.cos(z / 2)
    return np.mat([q0, q1, q2, q3])


def hat_map_33(mat):
    vec = np.mat([[0.0, -mat[2, 0], mat[1, 0]],
                  [mat[2, 0], 0.0, -mat[0, 0]],
                  [-mat[1, 0], mat[0, 0], 0.0]])
    return vec

def dcm_orthonormalize(R):
    x=np.array([R[0,0],R[0,1],R[0,2]])
    y=np.array([R[1,0],R[1,1],R[1,2]])
    error=np.dot(x,y)
    x_orthogonal = x - (0.5 * error * y)
    y_orthogonal = y - (0.5 * error * x)
    z_orthogonal = np.cross(x_orthogonal, y_orthogonal)
    x_normalized = 0.5 * (3 - np.dot(x_orthogonal, x_orthogonal)) * x_orthogonal
    y_normalized = 0.5 * (3 - np.dot(y_orthogonal, y_orthogonal)) * y_orthogonal
    z_normalized = 0.5 * (3 - np.dot(z_orthogonal, z_orthogonal)) * z_orthogonal
    R=np.vstack((x_normalized,np.vstack((y_normalized,z_normalized))))
    return R