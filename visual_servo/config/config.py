import numpy as np
#trajectory parameters
a_ = 8*10**(-4)  # 这个是图像特征的一个期望值
z_ = 4  # 期望高度，可能不应该是m
lambda_ = 3.2 #in mm
c1 = 0.1
c2 = 0.1
c3 = 1
k1=2
a=0.0064
b=-0.048
d=0.2

# 初始化变量
uf = np.array([0, 0, 0, 0]).reshape(4, 1)
q1_ = np.array([-1, -1, 1, 0]).reshape(4, 1)  # 特征的期望值
q1 = np.array([0, 0, 0, 0]).reshape(4, 1)

psi_d = 0
psi_ = 0
v = np.array([0, 0, 0]).reshape(3, 1)  # 虚坐标系下的速度