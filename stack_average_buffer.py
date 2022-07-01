import numpy as np
import math as m
RobotConfig_new = np.vstack((Pos, Vel, Angle, AngVel))
RobotConfig = np.hstack((RobotConfig, RobotConfig_new))


AngleBuffer  = np.hstack((AngleBuffer, newAngle))
# a = np.array([[1, 2, 3],
#               [1, 2, 3],
#               [1, 2, 3]]) # array
# print(np.repeat(a,2, axis=1))

# n = 4 # should be a multiple of the array
# a = np.array([[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
#               [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
#               [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]]) # array
# var1 = np.size(a,1)/n
# var2 = m.floor(var1) # take multiple of n from a
# var3 = var2*n
# var4 = round(n*(var1-var2))
# print(f"var1:{var1}\t" f"var2:{var2}\t" f"var3:{var3}\t" f"var4:{var4}")
#
# b = a[:, 0:-var4]
# print(b)
# mean_a = np.mean(b.reshape(-1, n), axis=1).reshape(3, -1)
# print(mean_a)

# a = np.array([[1, 2, 3],[4, 5, 6],[7, 8, 9]])
# print(a)
# np.mean(bufferSnippetDiff, axis=1, dtype=np.float64).T

# bufferSize = 10
# mu, sigma = 0, 0.1 # mean and standard deviation
# x = np.zeros((1,bufferSize + 2))
# y = np.zeros((1,bufferSize + 2))
# z = np.zeros((1,bufferSize + 2))
# xn = x + np.random.normal(mu,sigma,size=(1,bufferSize + 2))
# yn = y + np.random.normal(mu,sigma,size=(1,bufferSize + 2))
# zn = z + np.random.normal(mu,sigma,size=(1,bufferSize + 2))
#
# p = np.zeros((3,bufferSize + 2))
# pn = p + np.random.normal(mu,sigma,size=(3,bufferSize + 2))

# a = np.zeros((3,10 + 2))
# a[:, -1]= np.array([[1, 1, 1]])
# print(a)
# a = np.roll(a, -1)
# a[:, -1]= np.array([[2, 2, 2]])
# print(a)
# a = np.roll(a, -1)
# a[:, -1]= np.array([[3, 3, 3]])
# print(a)

# correctionBuffer = np.roll(correctionBuffer, -1)
#             correctionBuffer[:,[-1]] = correctionVectorGlobal