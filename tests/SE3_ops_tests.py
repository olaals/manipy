import sys
sys.path.insert(0,'..')
from manipy.SE3_ops import *
import spatialmath as sm
from liegroups import SE3 as lg_SE3

def vee_wedge():
    vec = [0.24,0.4,0.3,0.12,0.13,0.16]
    vec2 = vee(wedge(vec))
    passed = np.allclose(vec,vec2)
    assert passed

def compare_left_jacobian_lie_groups():
    vec = [0.24,0.4,0.3,0.12,0.13,0.16]
    left_jacobian = left_jacob(vec)
    left_jacobian_lg = lg_SE3.left_jacobian(vec)
    assert np.allclose(left_jacobian, left_jacobian_lg)

def compare_inv_left_jacob_lie_groups():
    vec = [0.24,0.4,0.3,0.12,0.13,0.16]
    inv_left_jacobian = inv_left_jacob(vec)
    inv_left_jacobian_lg = lg_SE3.inv_left_jacobian(vec)
    assert np.allclose(inv_left_jacobian, inv_left_jacobian_lg)


def compare_Exp_lie_groups():
    vec = [0.24,0.4,0.3,0.12,0.13,0.16]
    T = Exp(vec)
    T_lg = lg_SE3.exp(vec)
    assert np.allclose(T, T_lg)





def compare_sm_rx():
    T_sm = sm.SE3.Rx(1.3).data[0]
    T = Exp([0,0,0,1.3,0,0])
    assert np.allclose(T_sm, T)

def compare_sm_ry():
    T_sm = sm.SE3.Ry(1.3).data[0]
    T = Exp([0,0,0,0,1.3,0])
    assert np.allclose(T_sm, T)
    
def log_exp():
    theta = [0.24,0.4,0.3,0.12,0.13,0.16]
    theta2 = Log(Exp(theta))
    assert np.allclose(theta,theta2)

def jacob_mult_identity():
    theta = [0.24,0.4,0.3,0.12,0.13,0.16]
    left_jac = left_jacob(theta)
    inv_left_jac = inv_left_jacob(theta)
    assert np.allclose(left_jac@inv_left_jac, np.identity(6))






if __name__ == '__main__':
    vee_wedge()
    compare_sm_rx()
    compare_sm_ry()
    compare_left_jacobian_lie_groups()
    compare_inv_left_jacob_lie_groups()
    log_exp()
    jacob_mult_identity()








