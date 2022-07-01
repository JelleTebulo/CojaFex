import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

fig, (ax) = plt.subplots(subplot_kw=dict(projection="3d"))
c = 2*np.pi
tc = 0.0005
zc = 1
# z = zc * np.linspace(0, 2*c, 1000)
x = zc * np.linspace(0, 2*c, 1000)
z = c * np.cos(x)
y = c * np.sin(x)

plt.plot(x, y, z, lw=1, c='k', linestyle = '--')

def get_arrowy(i):
    x = zc * c * tc * i
    z = c * np.cos(x)
    y = c * np.sin(x)
    w = zc*c * - tc * np.sin(x)
    v = zc*c * tc * np.cos(x)
    u = zc*tc
    return x, y, z, u, v, w


def get_arrowz(i):
    x = zc * c * tc * i
    z = c * np.cos(x)
    y = c * np.sin(x)
    w = zc*c * - tc * np.sin(x)
    v = zc*c * tc * np.cos(x)
    u = zc*tc
    y_tcp = np.array([u, v, w])
    # x_g = np.array([[1, 0, 0]])
    # z_tcp = np.cross(x_g, y_tcp)
    # x_tcp = np.cross(y_tcp, z_tcp)
    z_g   = np.array([[0, 0, 1]])
    x_tcp = np.cross(y_tcp, z_g)
    z_tcp = np.cross(x_tcp, y_tcp)
    return x, y, z, z_tcp[0, 0], z_tcp[0, 1], z_tcp[0, 2]


def get_arrowx(i):
    x = zc * c * tc * i
    z = c * np.cos(x)
    y = c * np.sin(x)
    w = zc*c * - tc * np.sin(x)
    v = zc*c * tc * np.cos(x)
    u = zc*tc
    y_tcp = np.array([u, v, w])
    # x_g = np.array([[1, 0, 0]])
    # z_tcp = np.cross(x_g, y_tcp)
    # x_tcp = np.cross(y_tcp,z_tcp)
    z_g = np.array([[0, 0, 1]])
    x_tcp = np.cross(y_tcp, z_g)
    return x, y, z, x_tcp[0, 0], x_tcp[0, 1], x_tcp[0, 2]


quiver1 = ax.quiver(*get_arrowx(0))
quiver2 = ax.quiver(*get_arrowy(0))
quiver3 = ax.quiver(*get_arrowz(0))

ax.set_zlim(-c, c)
ax.set_zlim(-c, c)

if zc == 1:
    ax.set_xlim(0, zc*2*c)
else:
    ax.set_xlim(zc*2*c, 0)


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
# mng.full_screen_toggle()
plt.show()

# save animation at 30 frames per second
ani.save('myAnimation3.gif', writer='imagemagick', fps=30)


