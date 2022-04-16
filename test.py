from manipy import SO3, SE3
import numpy as np
import math


vec = np.array([1,2,3,math.pi/2,0.0, 0])
T_WD = SE3.from_vec(vec)
vec2 = np.array([0,1,0,0.0,0.0, 0])
ad = T_WD.adjoint()
T_WD2 = T_WD.perturb(vec2)
T_WD3 = T_WD.perturb_left(ad@vec2)
print(T_WD)
print(T_WD2)
print(T_WD3)


v = T_WD2.as_vec()
print(v)
print(ad@vec2)



