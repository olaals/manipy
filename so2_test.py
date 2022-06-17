from manipy import SO3
import spatialmath as sm
import math

print(sm.SE3.Rx(2))


a = SO3.from_Rx(2)
print(a)
