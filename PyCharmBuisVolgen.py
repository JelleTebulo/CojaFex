import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as R


def skew(vector):
    return np.array([[0, -vector[2][0], vector[1][0]],
                     [vector[2][0], 0, -vector[0][0]],
                     [-vector[1][0], vector[0][0], 0]])


def rodriguez(b, a):
    a = a / LA.norm(a)
    b = b / LA.norm(b)
    v = np.cross(a, b, axis=0)
    c = np.dot(a.T, b)
    # return np.eye(3) + skew(v) + LA.matrix_power(skew(v), 2) * (1 / (1 + c))
    return np.eye(3) + skew(v) + LA.matrix_power(skew(v), 2) * (1 / (1 + c))


# Rodriguez
a = np.array([[1, 2, 3]]).T
b = np.array([[4, 5, 6]]).T
a = np.array([[0, 1 ,0]]).T
b = np.array([[1e-3, 1, 1e-3]]).T
r = R.from_matrix(rodriguez(a, b))
eul = r.as_euler('zyx', degrees=True)
test = R.from_euler('zyx', [eul], degrees=True)
print(eul)
# test3 = test2.dot(b)
# test3 = test3/np.min(test3)
# print(test3)

# test1 = R.dot(b)
# test2 = test1 / np.min(test1)
# print(test2)

# # Lasers
# P_corr = np.array([[L_3 - L_1], [L_4-L_2]])
# R_y135 = array([[-1/np.sqrt(2),  0.        ,  1/np.sqrt(2)],
#                 [ 0.        ,  1.        ,  0.        ],
#                 [-1/np.sqrt(2),  0.        , -1/np.sqrt(2)]])
# P = P_curr+0.5*P_corr+P_offset
#
# # Shift measurements
# x_new = 6
# x = np.array([[5, 4, 3, 2, 1]])
# y = np.roll(x, 1)
# y[0][0] = x_new
# print(y)

