# PROC RotZYX(num psi, num th, num phi)
# ! Input angles in degrees to rotation matrix
#     R{1,1} := Cos(psi)*Cos(th);
#     R{2,1} := Cos(psi)*Sin(th)*Sin(phi)-Sin(psi)*Cos(phi);
#     R{3,1} := Cos(psi)*Sin(th)*Cos(phi)+Sin(psi)*Sin(phi);
#     R{1,2} := Sin(psi)*Cos(th);
#     R{2,2} := Sin(psi)*Sin(th)*Sin(phi)+Cos(psi)*Cos(phi);
#     R{3,2} := Sin(psi)*Sin(th)*Cos(phi)-Cos(psi)*Sin(phi);
#     R{1,3} := -Sin(th);
#     R{2,3} := Cos(th)*Sin(phi);
#     R{3,3} := Cos(th)*Cos(phi);
# ENDPROC

import numpy as np


def RotZYX(psi, th, phi):
    psi, th, phi = np.rad2deg(psi), np.rad2deg(th), np.rad2deg(phi)
    Rot = np.zeros((3, 3))
    R[1, 1] = np.cos(psi) * np.cos(th)
    R[2, 1] = np.cos(psi) * np.sin(th) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    R[3, 1] = np.cos(psi) * np.sin(th) * np.cos(phi) + np.sin(psi) * np.sin(phi)
    R[1, 2] = np.sin(psi) * np.cos(th)
    R[2, 2] = np.sin(psi) * np.sin(th) * np.sin(phi) + np.cos(psi) * np.cos(phi)
    R[3, 2] = np.sin(psi) * np.sin(th) * np.cos(phi) - np.cos(psi) * np.sin(phi)
    R[1, 3] = -np.sin(th)
    R[2, 3] = np.cos(th) * np.sin(phi)
    R[3, 3] = np.cos(th) * np.cos(phi)
    return R

psi, th, phi = 90, 0, 0
Yn = np.dot(Yo, RotZYX([psi, th, phi]))
