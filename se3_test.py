from manipy import SE3
import spatialmath as sm
import math
from liegroups import SE3 as lgSE3
from liegroups import SO3 as lgSO3

lg_T_rx = lgSE3(lgSO3.rotx(2), [1,2,3])
print(lg_T_rx)


T_rx = SE3.from_rx(2, [1,2,3])
print(T_rx)

print(T_rx.left_jacob())








