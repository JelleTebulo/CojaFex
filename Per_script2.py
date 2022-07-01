import numpy as np
from matplotlib import pyplot as plt
import scipy.signal
_T_s_ = 4e-3
_v_max_ = 10.
_a_max_ = 1.
class trajectory_generator():
    def __init__(self):
        self.prev_s_n = 0
        self.prev_d_lambda = 0
    def generate(self, P__1, P_0, P_1, P_2):
        dP_0_unscaled = (P_1 + P__1) / 2
        s_n =  np.linalg.norm(P_0 - P_1) / np.linalg.norm(dP_0_unscaled)
        dP_0 = s_n * dP_0_unscaled
        dP_1_unscaled = (P_2 + P_0) / 2
        s_n_1 = np.linalg.norm(P_1 - P_2) / np.linalg.norm(dP_1_unscaled)
        dP_1 = s_n_1 * dP_1_unscaled
        # find coefficients
        boundary = np.array([P_0, dP_0, P_1, dP_1])
        A = np.array([[0, 0, 0, 1], [0, 0, 1, 0], [1, 1, 1, 1], [3, 2, 1, 0]])
        coefs = np.linalg.pinv(A) @ boundary
        a = coefs[0]
        b = coefs[1]
        c = coefs[2]
        d = coefs[3]
        lambda_list = [0]
        d_lambda_list = [self.prev_d_lambda]
        k = 0
        while (lambda_list[k] < 1):
            p = a * lambda_list[-1] ** 3 + b * lambda_list[-1] ** 2 + c * lambda_list[-1] + d
            dp = 3 * a * lambda_list[-1] ** 2 + 2 * b * lambda_list[-1] + c
            D_end = np.linalg.norm(p - P_1)
            vel_current = d_lambda_list[-1] * np.linalg.norm(dp)
            d_lambda_max = _v_max_ / np.linalg.norm(dp)
            dd_lambda_max = _a_max_ / np.linalg.norm(dp)
            # d_lambda_next = 0
            #
            # if D_end <= ((vel_current**2) / 2 * _a_max_):
            #     vel_next = math.sqrt(0.5 * D_end * _a_max_)
            #     d_lambda_next = vel_next / np.linalg.norm(dp)
            # else:

            if d_lambda_list[-1] < d_lambda_max:
                d_lambda_next = d_lambda_list[-1] + _T_s_ * dd_lambda_max # d_lambda/dt k+1
            else:
                d_lambda_next = d_lambda_max

            lambda_list.append(lambda_list[k] + _T_s_ * d_lambda_next)
            d_lambda_list.append(d_lambda_next)
            self.prev_d_lambda = d_lambda_next
            k = k + 1

        lambd = np.array(lambda_list)
        d_lambda = np.array(d_lambda_list)
        P = a*lambd**3 + b*lambd**2 + c*lambd + d
        dP = 3 * a * lambd**2 + 2 * b*lambd + c
        v = dP * d_lambda
        self.prev_s_n = s_n
        return P, v, lambda_list, d_lambda_list


def lengthen_array(A, B, C):
    m = max(A.shape, B.shape, C.shape)
    m = int(''.join(map(str, m)))

    A_new = np.interp(np.linspace(1, C.size, m), np.linspace(1, A.size, A.size), A)
    B_new = np.interp(np.linspace(1, C.size, m), np.linspace(1, B.size, B.size), B)
    C_new = np.interp(np.linspace(1, C.size, m), np.linspace(1, C.size, C.size), C)
    return A_new, B_new, C_new

pos_bufferx = np.array([[0, 0, -5, -5, -5, -5]])
pos_buffery = np.array([[0, 0, 100, 200, 300, 400]])
pos_bufferz = np.array([[0, 0, 5, 5, 5, 5]])
traj_gen = trajectory_generator()
traject_pos_x_1, traject_vel_x_1, lambda_list_x_1, d_lambda_list_x_1 = traj_gen.generate(pos_bufferx[0, 0], pos_bufferx[0, 1], pos_bufferx[0, 2], pos_bufferx[0, 3]) # B -> C
traject_pos_y_1, traject_vel_y_1, lambda_list_y_1, d_lambda_list_y_1 = traj_gen.generate(pos_bufferx[0, 0], pos_bufferx[0, 1], pos_bufferx[0, 2], pos_bufferx[0, 3]) # B -> C
traject_pos_z_1, traject_vel_z_1, lambda_list_z_1, d_lambda_list_z_1 = traj_gen.generate(pos_bufferx[0, 0], pos_bufferx[0, 1], pos_bufferx[0, 2], pos_bufferx[0, 3]) # B -> C
traject_pos_x_2, traject_vel_x_2, lambda_list_x_2, d_lambda_list_x_2 = traj_gen.generate(pos_buffery[0, 1], pos_buffery[0, 2], pos_buffery[0, 3], pos_buffery[0, 4]) # C -> D
traject_pos_y_2, traject_vel_y_2, lambda_list_y_2, d_lambda_list_y_2 = traj_gen.generate(pos_buffery[0, 1], pos_buffery[0, 2], pos_buffery[0, 3], pos_buffery[0, 4]) # C -> D
traject_pos_z_2, traject_vel_z_2, lambda_list_z_2, d_lambda_list_z_2 = traj_gen.generate(pos_buffery[0, 1], pos_buffery[0, 2], pos_buffery[0, 3], pos_buffery[0, 4]) # C -> D
traject_pos_x_3, traject_vel_x_3, lambda_list_x_3, d_lambda_list_x_3 = traj_gen.generate(pos_bufferz[0, 2], pos_bufferz[0, 3], pos_bufferz[0, 4], pos_bufferz[0, 5]) # D -> E
traject_pos_y_3, traject_vel_y_3, lambda_list_y_3, d_lambda_list_y_3 = traj_gen.generate(pos_bufferz[0, 2], pos_bufferz[0, 3], pos_bufferz[0, 4], pos_bufferz[0, 5]) # D -> E
traject_pos_z_3, traject_vel_z_3, lambda_list_z_3, d_lambda_list_z_3 = traj_gen.generate(pos_bufferz[0, 2], pos_bufferz[0, 3], pos_bufferz[0, 4], pos_bufferz[0, 5]) # D -> E

traject_pos_x_1, traject_pos_y_1, traject_pos_z_1 = lengthen_array(traject_pos_x_1, traject_pos_y_1, traject_pos_z_1)
traject_pos_x_2, traject_pos_y_2, traject_pos_z_2 = lengthen_array(traject_pos_x_2, traject_pos_y_2, traject_pos_z_2)
traject_pos_x_3, traject_pos_y_3, traject_pos_z_3 = lengthen_array(traject_pos_x_3, traject_pos_y_3, traject_pos_z_3)
traject_vel_x_1, traject_vel_y_1, traject_vel_z_1 = lengthen_array(traject_vel_x_1, traject_vel_y_1, traject_vel_z_1)
traject_vel_x_2, traject_vel_y_2, traject_vel_z_2 = lengthen_array(traject_vel_x_2, traject_vel_y_2, traject_vel_z_2)
traject_vel_x_3, traject_vel_y_3, traject_vel_z_3 = lengthen_array(traject_vel_x_3, traject_vel_y_3, traject_vel_z_3)

traject_pos_1 = np.stack((traject_pos_x_1, traject_pos_y_1, traject_pos_z_1), axis=1)
traject_pos_2 = np.stack((traject_pos_x_2, traject_pos_y_2, traject_pos_z_2), axis=1)
traject_pos_3 = np.stack((traject_pos_x_3, traject_pos_y_3, traject_pos_z_3), axis=1)
traject_vel_1 = np.stack((traject_vel_x_1, traject_vel_y_1, traject_vel_z_1), axis=1)
traject_vel_2 = np.stack((traject_vel_x_2, traject_vel_y_2, traject_vel_z_2), axis=1)
traject_vel_3 = np.stack((traject_vel_x_3, traject_vel_y_3, traject_vel_z_3), axis=1)

trajectory = np.concatenate((traject_pos_1, traject_pos_2, traject_pos_3), axis=0)
velocity = np.concatenate((traject_vel_1, traject_vel_2, traject_vel_3), axis=0)
time_vector = np.arange(0, trajectory.shape[0]) * _T_s_
norm_vel = np.linalg.norm(velocity, axis=1)
fig, ((ax1, ax2, norm), (ax3, ax4, ax5)) = plt.subplots(2, 3)
ax1.set_title('position')
nu = 1
ax1.plot(time_vector[:-nu], trajectory[:-nu, 0], label='x')
ax1.plot(time_vector[:-nu], trajectory[:-nu, 1], label='y')
ax1.plot(time_vector[:-nu], trajectory[:-nu, 2], label='z')
ax2.set_title('velocity')
ax2.plot(time_vector[:-nu], velocity[:-nu, 0], label='x')
ax2.plot(time_vector[:-nu], velocity[:-nu, 1], label='y')
ax2.plot(time_vector[:-nu], velocity[:-nu, 2], label='z')
norm.plot(time_vector, norm_vel)
plt.show()