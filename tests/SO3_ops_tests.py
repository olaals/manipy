import sys
sys.path.insert(0,'..')
from manipy.SO3_ops import *
import spatialmath as sm

def vee_wedge():
    theta = [0.24,0.4,0.3]
    theta2 = vee(wedge(theta))
    passed = np.allclose(theta,theta2)
    assert passed

def log_exp():
    theta = [0.2,0.4,0.5]
    theta2 = Log(Exp(theta))
    assert np.allclose(theta,theta2)

def compare_sm_rx():
    R_sm = sm.SO3.Rx(1.3).data[0]
    R = Exp([1.3,0,0])
    assert np.allclose(R_sm, R)

def compare_sm_ry():
    R_sm = sm.SO3.Ry(1.3).data[0]
    R = Exp([0,1.3,0])
    assert np.allclose(R_sm, R)

def jacobian_mult_identity():
    theta = [0.2,0.4,0.5]
    left_jac = left_jacob(theta)
    inv_left_jac = inv_left_jacob(theta)
    res = left_jac@inv_left_jac
    assert np.allclose(res, np.identity(3))

    
    





if __name__ == '__main__':
    vee_wedge()
    log_exp()
    compare_sm_rx()
    compare_sm_ry()
    jacobian_mult_identity()








