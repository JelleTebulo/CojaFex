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

data = np.array(pd.read_csv('Data/DataBuisError.csv'))

fig, (ax) = plt.subplots(subplot_kw=dict(projection="3d"))
c = 2*np.pi
tc = 0.0005
zc = 1
x = data[:, 0]
z = data[:, 1]
y = data[:, 2]
u1 = np.diff(x)
v1 = np.diff(y)
w1 = np.diff(z)
u = u1/np.linalg.norm([u1, v1, w1], axis=0)
v = v1/np.linalg.norm([u1, v1, w1], axis=0)
w = w1/np.linalg.norm([u1, v1, w1], axis=0)

ax.plot(x, y, z, lw=1, c='k', linestyle = '--')

fig2, ax2 = plt.subplots()
ax2.plot(u1, lw=1, c='b', linestyle = '-')
ax2.plot(v1, lw=1, c='c', linestyle = '-')
ax2.plot(w1, lw=1, c='k', linestyle = '-')
ax2.plot(u, lw=1, c='b', linestyle = ':')
ax2.plot(v, lw=1, c='c', linestyle = ':')
ax2.plot(w, lw=1, c='k', linestyle = ':')
ax2.plot(np.linalg.norm([u,v,w], axis=0), lw=1, c='g', linestyle = '-')


def get_arrowy(i):
    return x[i], y[i], z[i], u[i], v[i], w[i]

def get_arrowz(i):
    y_tcp = np.array([u[i], v[i], w[i]])
    z_g   = np.array([[0, 0, 1]])
    x_tcp = np.cross(y_tcp, z_g)
    z_tcp = np.cross(x_tcp, y_tcp)
    return x[i], y[i], z[i], z_tcp[0, 0], z_tcp[0, 1], z_tcp[0, 2]

def get_arrowx(i):
    y_tcp = np.array([u[i], v[i], w[i]])
    z_g = np.array([[0, 0, 1]])
    x_tcp = np.cross(y_tcp, z_g)
    return x[i], y[i], z[i], x_tcp[0, 0], x_tcp[0, 1], x_tcp[0, 2]


quiver1 = ax.quiver(*get_arrowx(0))
quiver2 = ax.quiver(*get_arrowy(0))
quiver3 = ax.quiver(*get_arrowz(0))

# ax.set_zlim(-c, c)
# ax.set_zlim(-c, c)
#
# if zc == 1:
#     ax.set_xlim(0, zc*2*c)
# else:
#     ax.set_xlim(zc*2*c, 0)


def update(i):
    global quiver1
    global quiver2
    global quiver3
    quiver1.remove()
    quiver2.remove()
    quiver3.remove()
    quiver1 = ax.quiver(*get_arrowx(i), color='g', length=c/2, normalize=True, label='x')
    quiver2 = ax.quiver(*get_arrowy(i), color='c', length=c/2, normalize=True, label='y')
    quiver3 = ax.quiver(*get_arrowz(i), color='b', length=c/2, normalize=True, label='z')
    ax.legend()


ani = FuncAnimation(fig, update, interval=1, repeat=True, save_count = 400)
ax.set_xlabel('X(t)')
ax.set_ylabel('Y(t)')
ax.set_zlabel('Z(t)')

mng = plt.get_current_fig_manager()
plt.show()

# save animation at 30 frames per second
ani.save('MyAnimation.gif', writer='imagemagick', fps=30)


