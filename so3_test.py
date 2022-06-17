from manipy import SO3
import spatialmath as sm
import math
from liegroups import SO3 as lgSO3

print(sm.SO3.Rx(2))
print(SO3.from_rx(2))
print(sm.SO3.Ry(2))
print(SO3.from_ry(2))
print(sm.SO3.Rz(2))
print(SO3.from_rz(2))

print("As logarithm")
print(SO3.from_rx(2).as_logarithm())

lg_rx = lgSO3.rotx(2)
rx = SO3.from_rx(2)
print(rx)
print(lg_rx)

lj = rx.left_jacob()
vec = rx.as_vec()
lg_lj = lg_rx.left_jacobian(vec)
print(lj)
print(lg_lj)






