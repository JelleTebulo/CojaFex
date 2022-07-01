import math
import numpy as np
import random
# print("1: ", math.degrees(math.atan2(1,1)))
# print("2: ", math.degrees(math.atan2(1,-1)))
# print("3: ", math.degrees(math.atan2(-1,-1)))
# print("4: ", math.degrees(math.atan2(-1,1)))

r          = 1e-5
n          = 10
x          = 2*np.array([1, -1, -1,  1])
y          = 2*np.array([1,  1, -1, -1])
z          = np.array([0,  0,  0,  0])
sum_lxy, \
sum_lz, \
sum_lxy_lz, \
sum_lxy2, \
sum_lx, \
sum_ly, \
sum_lx_ly, \
sum_ly2 = 0, 0 ,0 ,0 ,0 ,0 ,0 ,0

for a in range(np.size(x)):
    for b in range(n):
        lx          = x[a] + r #+ np.random.uniform(low=-r, high=r, size=None)
        ly          = y[a] + r #+ np.random.uniform(low=-r, high=r, size=None)
        lz          = z[a] + r #+ np.random.uniform(low=-r, high=r, size=None)
        # print("lx = ", lx, "lx-x = ", lx-x[a])
        # print("ly = ", ly, "lx-x = ", ly-y[a])
        # print("lz = ", lz, "lx-x = ", lz-z[a])
        lxy         = math.sqrt(lx**2 + ly**2)
        # print("lxy = ", lxy)
        sum_lxy    += lxy
        sum_lz     += lz
        sum_lxy_lz += lxy*lz
        sum_lxy2   += lxy**2
        sum_lx     += lx
        sum_ly     += ly
        sum_lx_ly  += lx*ly
        sum_ly2    += ly**2
    rc_line_rx = ((n*sum_lxy_lz)-(sum_lxy*sum_lz))/((n*sum_lxy2)-(sum_lxy**2))
    rc_line_rz = ((n*sum_lx_ly)-(sum_ly*sum_lx))/((n*sum_ly2)-(sum_ly**2))
    # print("n = ",n," sum_lx_ly = ",sum_lx_ly," sum_lx = ", sum_lx, " sum_ly = ",sum_ly, " sum_lz = ",sum_lz," n*sum_ly2 = ", n*sum_ly2, " sum_ly**2 = ", sum_ly**2)
    rx         = math.atan2(rc_line_rx,1)
    if sum_ly < 0:
        rz = math.atan2(rc_line_rz,-1)
    else:
        rz = math.atan2(rc_line_rz,1)
        print()
    print("\nFor QUADRANT",a+1,"           x=", x[a]," y=", y[a]," z=", z[a])
    print("###########################################################################")
    print("rc_line_rz = ", round(rc_line_rz,2)      , "        rc_line_rx = ", round(rc_line_rx,2))
    print("rz         = ", round(math.degrees(rz),2), "        rx         = ", round(math.degrees(rx),2))

