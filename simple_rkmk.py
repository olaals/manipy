from manipy import SE3_ops, SE3
import numpy as np
import math
import matplotlib.pyplot as plt


def kinematics(_, c, h):
    vec = SE3.from_rz(math.pi, t=[0.1,0,0]).as_vec()
    return vec


h = 0.05

T0 = SE3.identity()
Ts = []
Ts.append(T0)

T=T0
xs = []
ys = []
xs.append(T0.t[0])
ys.append(T0.t[1])

for i in range(20):
    c=0
    theta = h*kinematics(0, c, h)
    F = SE3_ops.inv_right_jacob(np.zeros(6))@theta
    #F = np.identity(6)@theta
    Theta = 1.0*F
    T = T.perturb(Theta)
    xs.append(T.t[0])
    ys.append(T.t[1])

print(xs,ys)
plt.plot(xs,ys)
plt.show()




    
    







