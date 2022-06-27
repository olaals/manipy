from manipy import SE2_ops
import numpy as np
import matplotlib.pyplot as plt

def constant_dynamics(SE2_mat, time):
    return np.array([0.1,.0,0.1])



T_0 = np.identity(3)

xs = []
ys = []
steps = 100

T = T_0

for i in range(10):
    u = constant_dynamics(0,0)
    T = T@SE2_ops.Exp(u)
    xs.append(T[0,2])
    ys.append(T[1,2])
    print(T)

plt.plot(xs,ys)
plt.show()
    


