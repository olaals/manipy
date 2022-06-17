from manipy import SE3
import matplotlib.pyplot as plt


p_vec = SE3.from_rz(0.1, [0.1,0,0])
print("perturb vector")
print(p_vec)


T0 = SE3.identity()
print(T0)

steps = 63
T = T0
Ts = []
Ts.append(T)
xs = []
ys = []

for i in range(steps):
    T = T*p_vec
    print(T)
    Ts.append(T)
    xs.append(T.t[0])
    ys.append(T.t[1])

plt.plot(xs,ys)
plt.show()






