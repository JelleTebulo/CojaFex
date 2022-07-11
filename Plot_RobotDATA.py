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

data = np.array(pd.read_csv('Data/TargetNewData_11_07_22.csv'))

# fig, (ax) = plt.subplots(subplot_kw=dict(projection="3d"))
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
u1 = np.diff(x)
v1 = np.diff(y)
w1 = np.diff(z)
u = u1 / np.linalg.norm([u1, v1, w1], axis=0)
v = v1 / np.linalg.norm([u1, v1, w1], axis=0)
w = w1 / np.linalg.norm([u1, v1, w1], axis=0)

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


fig3, axs3 = plt.subplots(2, 3, sharex=True)
end = 291
rx = data[:end, 3]
ry = data[:end, 4]
rz = data[:end, 5]
axs3[0, 0].plot(rx, c='b', linestyle='-')
axs3[0, 1].plot(ry, c='c', linestyle='-')
axs3[0, 2].plot(rz, c='k', linestyle='-')
drx = np.diff(data[:end, 3])
dry = np.diff(data[:end, 4])
drz = np.diff(data[:end, 5])
axs3[1, 0].plot(drx, c='b', linestyle='-')
axs3[1, 1].plot(dry, c='c', linestyle='-')
axs3[1, 2].plot(drz, c='k', linestyle='-')


# for th in range(0, 180, 20):
#     th = np.deg2rad(th)
#     a = np.array([np.cos(th),np.sin(th),0])
#     b = np.array([-np.sin(th),np.cos(th),0])
#     a_s = a/np.linalg.norm(a)
#     b_s = b/np.linalg.norm(b)
#     c = np.cross(a_s, b_s, axis=0)
#     c_s = c/ np.linalg.norm(c)
#     print(f'a =', np.linalg.norm(a_s), '          b =', np.linalg.norm(b_s), '          c =', np.linalg.norm(c))