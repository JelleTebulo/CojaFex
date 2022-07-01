import numpy as np
import matplotlib.pyplot as plt

def horizontal_spiral():
    c  = 2*np.pi
    x  = np.linspace(0,2*c,t_end)
    y  = c * np.sin(x)
    z  = c * np.cos(x)
    dx = dt*np.ones(np.size(x))
    dy = dt*c*np.cos(x)
    dz = -dt*c*np.sin(x)
    L = c/2
    return x,y,z,dx,dy,dz,L

def vertical_spiral():
    c = 2 * np.pi
    z = np.linspace(0, 2 * c, t_end)
    y = c * np.sin(z)
    x = c * np.cos(z)
    dz = dt * np.ones(np.size(z))
    dy = dt * c * np.cos(z)
    dx = -dt * c * np.sin(z)
    L = c / 2
    return x,y,z,dx,dy,dz,L

fig, (ax) = plt.subplots(subplot_kw=dict(projection="3d"))
t  = 1
dt = 0.01
t_end = int(t/dt)
x,y,z,dx,dy,dz,L = horizontal_spiral()
# x,y,z,dx,dy,dz,L = vertical_spiral()

# for i in range(int(t/dt)):
#     y_tcp = np.array([dx[i], dy[i], dz[i]])
#     # x_g = np.array([[1, 0, 0]])
#     # z_tcp = np.cross(x_g, y_tcp)
#     # x_tcp = np.cross(y_tcp, z_tcp)
#     z_g   = np.array([[0, 0, 1]])
#     x_tcp = np.cross(y_tcp, z_g)
#     z_tcp = np.cross(x_tcp, y_tcp)
#     quiver1 = ax.quiver(x[i], y[i], z[i], x_tcp[0, 0], x_tcp[0, 1], x_tcp[0, 2],  color='g', length=L, normalize=True, label='x')
#     quiver2 = ax.quiver(x[i], y[i], z[i], dx[i], dy[i], dz[i], color='c', length=L, normalize=True, label='y')
#     quiver3 = ax.quiver(x[i], y[i], z[i], z_tcp[0, 0], z_tcp[0, 1], z_tcp[0, 2], color='b', length=L, normalize=True, label='z')
#     plt.plot(x, y, z, lw=1, c='k', linestyle='--')
#     ax.legend()
#     plt.pause(dt)
#     plt.cla()

for i in range(int(t/dt)):
    y_tcp = np.array([dx[i], dy[i], dz[i]])
    # x_g ##################################
    # x_g = np.array([[1, 0, 0]])
    # z_tcp = np.cross(x_g, y_tcp)
    # x_tcp = np.cross(y_tcp, z_tcp)
    # z_g ##################################
    # z_g   = np.array([[0, 0, 1]])
    # x_tcp = np.cross(y_tcp, z_g)
    # z_tcp = np.cross(x_tcp, y_tcp)
    # origin ##################################
    z_tcp = np.cross(np.array([x[i],y[i],z[i]]),y_tcp)
    x_tcp = np.cross(y_tcp, z_tcp)

    quiver1 = ax.quiver(x[i], y[i], z[i], x_tcp[0, 0], x_tcp[0, 1], x_tcp[0, 2],  color='g', length=L, normalize=True, label='x')
    quiver2 = ax.quiver(x[i], y[i], z[i], dx[i], dy[i], dz[i], color='c', length=L, normalize=True, label='y')
    quiver3 = ax.quiver(x[i], y[i], z[i], z_tcp[0, 0], z_tcp[0, 1], z_tcp[0, 2], color='b', length=L, normalize=True, label='z')
    plt.plot(x, y, z, lw=1, c='k', linestyle='--')
    ax.legend()
    plt.pause(dt)
    plt.cla()

plt.show()
ax.set_xlabel('X(t)')
ax.set_ylabel('Y(t)')
ax.set_zlabel('Z(t)')
