import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

z = 0
x = 0
y = 0

x_g = np.array([[1, 0, 0]])
y_g = np.array([[0, 1, 0]])
z_g = np.array([[0, 0, 1]])

y_tcp = np.array([[.01, .01, 1]])
# z_tcp = np.cross(x_g, y_tcp)
# x_tcp = np.cross(y_tcp, z_tcp)
x_tcp = np.cross(y_tcp,z_g)
z_tcp = np.cross(x_tcp, y_tcp)

line1 = plt.plot(x, y, z, lw=1, c='k')
quiver1 = ax.quiver(x, y, z, x_tcp[0, 0], x_tcp[0, 1], x_tcp[0, 2], color='r', length=1, normalize=True, label='x')
quiver2 = ax.quiver(x, y, z, y_tcp[0, 0], y_tcp[0, 1], y_tcp[0, 2], color='g', length=1, normalize=True, label='y')
quiver3 = ax.quiver(x, y, z, z_tcp[0, 0], z_tcp[0, 1], z_tcp[0, 2], color='b', length=1, normalize=True, label='z')

quiver1 = ax.quiver(x, y, z, x_g[0, 0], x_g[0, 1], x_g[0, 2], color='r', linestyle=':', length=1, normalize=True)
quiver2 = ax.quiver(x, y, z, y_g[0, 0], y_g[0, 1], y_g[0, 2], color='g', linestyle=':', length=1, normalize=True)
quiver3 = ax.quiver(x, y, z, z_g[0, 0], z_g[0, 1], z_g[0, 2], color='b', linestyle=':', length=1, normalize=True)

ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)
ax.set_xlabel('X(t)')
ax.set_ylabel('Y(t)')
ax.set_zlabel('Z(t)')

ax.legend()
mng = plt.get_current_fig_manager()
# mng.full_screen_toggle()
plt.show()


