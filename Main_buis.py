import time

from rwsuis import RWS
import time
import numpy as np
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


def RotMat_2d(theta):
    theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    return np.array(((c, -s), (s, c)))

import os
import sys
# --- Config
robot_ip = '127.0.0.1'

# ---Setup RWS conn
http_addr = f'http://{robot_ip}'
rws_conn = RWS.RWS(http_addr)
rws_conn.request_mastership()

x_offset = 0
y_offset = 0
P_offset = np.array([[x_offset, y_offset]]).T

corrDataList = []

# --- Testing
while True:

    value = rws_conn.get_rapid_variable('Lasers')
    currentPos = rws_conn.get_tcp_position('tInductionCoil')

    currentTrans, currentRot = currentPos
    # [w, x, y, z] quat from RobotStudio

    # swap quat real part  scipy wil [x, y, z, w]
    currentRot = np.roll(currentRot, -1)

    # local transformations (relative to current tcp)
    dX = 0.0
    dY = -100.0
    dZ = 0.0
    dRx = 0.0
    dRy = -45.0
    dRz = 0.0

    Rbase_tcp = R.from_quat(currentRot).as_matrix()
    Ptarget = np.array([[dX, dY, dZ]]).T
    Rtcp_target = R.from_euler('zyx',  [dRz, dRy, dRx], degrees=True).as_matrix()
    Rbase_target = Rbase_tcp.dot(Rtcp_target)

    targetRot = R.from_matrix(Rbase_target).as_quat()
    targetRot = np.roll(targetRot, 1)

    targetTrans = currentTrans.T + Rbase_target.dot(Ptarget)

    rws_conn.set_robtarget('pTargetPosition', targetTrans.flatten().tolist(), targetRot.flatten().tolist())

    #rws_conn.set_robtarget_translation('pTargetPosition', [currentTrans[0], currentTrans[1]+100, currentTrans[2]])


    exit()





    value = value.replace('[', '').replace(']', '')
    laserValues = np.fromstring(value, dtype=float, sep=',')

    # Lasers
    P_corr = np.array([[laserValues[2] - laserValues[0]], [laserValues[3] - laserValues[1]]])
    R_y = RotMat_2d(-45)
    P = 0.5 * (R_y.dot(P_corr)) + P_offset

    if any(i >= 250 for i in laserValues): # break if any sensor values are out of range
        break

    corrDataList.append(P)
    #print(f"TCP correction X: {P[0][0]:.2f}\tZ:{P[1][0]:.2f}")

    time.sleep(0.5)



angleList =[]

for i in range(0, len(corrDataList)-2):
    a = np.vstack((corrDataList[i], [0]))
    b = np.vstack(((corrDataList[i] + corrDataList[i+1] + corrDataList[i+2])/3, [0]))
    r = R.from_matrix(rodriguez(a, b))
    eul = r.as_euler('zyx', degrees=True)

    angleList.append(eul)

# # Rodriguez
# a = np.array([[1, 2, 0]]).T
# b = np.array([[4, 5, 0]]).T
#
# eul = r.as_euler('zyx', degrees=True)

