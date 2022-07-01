import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R


def skew(vector):
    return np.array([[0, -vector[2][0], vector[1][0]],
                     [vector[2][0], 0, -vector[0][0]],
                     [-vector[1][0], vector[0][0], 0]])


def rodriguez(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b, axis=0)
    c = np.dot(a.T, b)
    return np.eye(3) + skew(v) + np.linalg.matrix_power(skew(v), 2) * (1 / (1 + c))


# a = np.array([[1, 2, 3]]).T
# b = np.array([[0, 1, 0]]).T
# R = rodriguez(b, a)
# a = np.dot(R, b)
# a = a / np.min(a)
# print(a)


f = pd.read_excel(r'D:\OneDrive - Tebulo Facilities B.V\Buis volgen P21-100392\Data.xlsx').to_numpy()


x  = f[6, 0]
y  = f[6, 1]
z  = f[6, 2]
rx = f[6, 3]
ry = f[6, 4]
rz = f[6, 5]
vx_wo = f[6, 9]
vy_wo = f[6, 10]
vz_wo = f[6, 11]

r = R.from_euler('zyx', [rx, ry, rz], degrees=True)
Rot = r.as_matrix()
tcp_l = np.array([[0, 1, 0]]).T
tcp_g = np.dot(Rot,tcp_l)
wo_g  = np.array([vx_wo, vy_wo, vz_wo])

Rot_r = rodriguez(tcp_g, wo_g)
test = np.dot(Rot_r, tcp_g)
test = test / np.min(test)
wo_g = wo_g / np.min(wo_g)
print(test)
print(wo_g)