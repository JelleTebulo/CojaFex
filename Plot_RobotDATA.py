# # def rCalcOrient()
#         VAR num reg_vector_xyz := 0;
#         VAR pos y_tcp := [0,0,0];
#         VAR pos x_tcp := [0,0,0];
#         VAR pos z_tcp := [0,0,0];
#         VAR pos z_g := [0,0,1];
#         VAR num rX_Calc := 0;
#         VAR num rY_Calc := 0;
#         VAR num rZ_Calc := 0;
#         VAR num startPos := 0;
#         VAR num eindPos := 0;
#         nOrientFilterCounter := nPosFilterCounter-((ORIENT_FILTER_SIZE-1)*0.5);
#         startPos := nOrientFilterCounter-((ORIENT_FILTER_SIZE-1)*0.5);
#         eindPos := nOrientFilterCounter+((ORIENT_FILTER_SIZE-1)*0.5);
#
#         rLinearFilterRot(startPos),(eindPos);
#
#         reg_vector_xyz := sqrt(pow((OrientVectorEind.x-OrientVectorStart.x),2)+pow((OrientVectorEind.y-OrientVectorStart.y),2)+pow((OrientVectorEind.z-OrientVectorStart.z),2));
#
#         y_tcp.x := (OrientVectorEind.x-OrientVectorStart.x)/reg_vector_xyz;
#         y_tcp.y := (OrientVectorEind.y-OrientVectorStart.y)/reg_vector_xyz;
#         y_tcp.z := (OrientVectorEind.z-OrientVectorStart.z)/reg_vector_xyz;
#
#         ! kruisproduct van de vectoren om de vector haaks op het vlak te berekenen
#         ! kruisproduct van y_tcp met z_g om de x_tcp te bepalen
#         ! kruisproduct van x_tcp met y_tcp om de z_tcp te bepalen
#         ! tesamen is het: Rbase_tcp_g = [x_tcp, y_tcp, z_tcp]
#         x_tcp.x := ((y_tcp.y*z_g.z)-(z_g.y*y_tcp.z));
#         x_tcp.y := -((y_tcp.x*z_g.z)-(z_g.x*y_tcp.z));
#         x_tcp.z := ((y_tcp.x*z_g.y)-(z_g.x*y_tcp.y));
#         z_tcp.x := ((x_tcp.y*y_tcp.z)-(y_tcp.y*x_tcp.z));
#         z_tcp.y := -((x_tcp.x*y_tcp.z)-(y_tcp.x*x_tcp.z));
#         z_tcp.z := ((x_tcp.x*y_tcp.y)-(y_tcp.x*x_tcp.y));
#
#         ! Array: [x_tcp , y_tcp, z_tcp]
#         rX_Calc := asin(y_tcp.z);
#         IF z_tcp.z<>0 THEN
#             rY_Calc := atan2(-x_tcp.z,z_tcp.z);
#         ELSE
#             rY_Calc := 0;
#         ENDIF
#         IF y_tcp.y<>0 THEN
#             rZ_Calc := atan2(-y_tcp.x,y_tcp.y);
#         ELSE
#             rZ_Calc := 0;
#         ENDIF
#
#         TargetNew{nOrientFilterCounter}.rot := OrientZYX(rZ_Calc,rY_Calc,rX_Calc)*OrientZYX(0,Rtcpy_Offset,0);
#
#         pnewCalc := [rZ_Calc,rY_Calc,rX_Calc];


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import pandas as pd

import math

def euler_yzx_to_axis_angle(z_e, x_e, y_e, normalize=True):
    # Assuming the angles are in radians.
    c1 = math.cos(z_e/2)
    s1 = math.sin(z_e/2)
    c2 = math.cos(x_e/2)
    s2 = math.sin(x_e/2)
    c3 = math.cos(y_e/2)
    s3 = math.sin(y_e/2)
    c1c2 = c1*c2
    s1s2 = s1*s2
    w = c1c2*c3 - s1s2*s3
    x = c1c2*s3 + s1s2*c3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3
    angle = 2 * math.acos(w)
    if normalize:
        norm = x*x+y*y+z*z
        if norm < 0.001:
            # when all euler angles are zero angle =0 so
            # we can set axis to anything to avoid divide by zero
            x = 1
            y = 0
            z = 0
        else:
            norm = math.sqrt(norm)
            x /= norm
            y /= norm
            z /= norm
    return z, x, y, angle


data = np.array(pd.read_csv('Data/TargetNew_11-7-22-1216.csv'))
data = np.array(pd.read_csv('Data/TargetNew_11-7-22-1154.csv'))
data = np.array(pd.read_csv('Data/TargetNew_11-7-22-1326.csv'))


x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
u1 = np.diff(x)
v1 = np.diff(y)
w1 = np.diff(z)
u = u1 / np.linalg.norm([u1, v1, w1], axis=0)
v = v1 / np.linalg.norm([u1, v1, w1], axis=0)
w = w1 / np.linalg.norm([u1, v1, w1], axis=0)

# fig, (ax) = plt.subplots(subplot_kw=dict(projection="3d"))
# ax.plot(x, y, z, lw=1, c='k', linestyle = '--')
#
# fig2, ax2 = plt.subplots()
# ax2.plot(u1, lw=1, c='b', linestyle = '-')
# ax2.plot(v1, lw=1, c='c', linestyle = '-')
# ax2.plot(w1, lw=1, c='k', linestyle = '-')
# ax2.plot(u, lw=1, c='b', linestyle = ':')
# ax2.plot(v, lw=1, c='c', linestyle = ':')
# ax2.plot(w, lw=1, c='k', linestyle = ':')
# ax2.plot(np.linalg.norm([u,v,w], axis=0), lw=1, c='g', linestyle = '-')

axis_angle = np.zeros(np.shape(data)[0])
from scipy.spatial.transform import Rotation as R

rx = data[:, 3]
ry = data[:, 4]
rz = data[:, 5]
drx = np.diff(data[:, 3])
dry = np.diff(data[:, 4])
drz = np.diff(data[:, 5])
for i in range(len(drx)):
    z, x, y, angle = euler_yzx_to_axis_angle(drz[i], drx[i], dry[i], normalize=True)
    axis_angle[i] = angle

beg, end = 0, 1300
FS = 15
fig3, axs3 = plt.subplots(2, 4, sharex=True)
axs3[0, 0].plot(rx[beg:end], c='b', linestyle='-')
axs3[0, 1].plot(ry[beg:end], c='c', linestyle='-')
axs3[0, 2].plot(rz[beg:end], c='k', linestyle='-')
axs3[0, 0].set_xlabel('$rx$', fontsize=FS)
axs3[0, 1].set_xlabel('$ry$', fontsize=FS)
axs3[0, 2].set_xlabel('$rz$', fontsize=FS)
axs3[1, 0].plot(drx[beg:end], c='b', linestyle='-')
axs3[1, 1].plot(dry[beg:end], c='c', linestyle='-')
axs3[1, 2].plot(drz[beg:end], c='k', linestyle='-')
axs3[1, 0].set_xlabel('$drx$', fontsize=FS)
axs3[1, 1].set_xlabel('$dry$', fontsize=FS)
axs3[1, 2].set_xlabel('$drz$', fontsize=FS)

axs3[1, 3].plot(axis_angle[beg:end], c='k', linestyle='-')
axs3[1, 3].set_xlabel('$d axis angle$', fontsize=FS)
axs3[0, 3].plot(np.diff(data[beg:end, 3]), c='b', linestyle='-')
axs3[0, 3].plot(np.diff(data[beg:end, 4]), c='c', linestyle='-')
axs3[0, 3].plot(np.diff(data[beg:end, 5]), c='k', linestyle='-')
axs3[0, 3].set_xlabel('$rx, ry, rz$', fontsize=FS)


