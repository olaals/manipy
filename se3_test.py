from manipy import SE3, SE3_ops, SO3_ops
import spatialmath as sm
import math
from liegroups import SE3 as lgse3
from liegroups import SO3 as lgso3
import numpy as np

def t1():
    # Log/Exp tests
    vec = np.array([1,2,1,np.pi/2.0, .0, .0])
    T1 = SE3.from_vec(vec)
    vec2 = SE3.as_vec(T1)
    print(vec)
    print(vec2)
    assert np.allclose(vec, vec2), "SE3_Exp(SE3_Log(vec)) != vec"




if __name__ == '__main__':
    t1()
    












