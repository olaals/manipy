from . import SO3_ops
from .utils import *

def decompose_Rt(se3_mat):
    assert se3_mat.shape == (4,4)
    R = se3_mat[0:3,0:3]
    t = se3_mat[0:3,3]
    return R, t

def wedge(vec):
    assert vec.shape == (6,)
    SE3_lie_alg = np.zeros((4,4))
    rho = vec[0:3]
    theta = vec[3:6]
    SO3_lie_alg = SO3_wedge(theta)
    SE3_lie_alg[0:3,0:3] = SO3_lie_alg
    SE3_lie_alg[0:3,3] = rho
    return SE3_lie_alg

def vee(lie_alg):
    assert lie_alg.shape == (4,4)
    SO3_lie_alg = lie_alg[0:3,0:3]
    rho = lie_alg[0:3,3]
    omega = SO3_ops.vee(SO3_lie_alg)
    return np.concatenate((rho,omega))

def Exp(vec):
    assert vec.shape == (6,)
    transf_mat = np.identity(4)
    rho = vec[0:3]
    theta = vec[3:6]
    rot_mat = SO3_ops.Exp(theta)
    transf_mat[0:3,0:3] = rot_mat
    transf_mat[0:3,3] = SO3_ops.left_jacob(theta) @ rho
    return transf_mat

def exp(lie_alg):
    assert lie_alg.shape == (4,4)
    transf_mat = np.identity(4)
    u_skew = lie_alg[:3,:3]
    rho = lie_alg[:3, 3]
    R = SO3_ops.exp(u_skew)
    transf_mat[0:3,0:3] = R
    transf_mat[0:3,3] = SO3_ops.left_jacob(theta) @ rho
    return transf_mat

def Log(se3_mat):
    assert transf_mat.shape == (4,4)
    R,t = SE3_decompose(transf_mat)
    theta = SO3_Log(R)
    rho = SO3_inv_left_jacob(theta) @ t
    rho = np.squeeze(rho)
    return np.concatenate((rho,theta))

def log(se3_mat):
    return exp(Log(se3_mat))

def adjoint(se3_mat):
    ad = np.zeros((6,6))
    rot_mat, transl = decompose_Rt(se3_mat)
    skew_t = SO3.wedge(transl)
    ad[:3,:3] = rot_mat
    ad[:3, 3:6] = skew_t@rot_mat
    ad[3:6, 3:6] = rot_mat
    return ad

def jacob_q_term(vec):
    assert vec.shape == (6,)
    rho = vec[:3] # rho
    th = vec[3:] # theta
    an = np.linalg.norm(th) # angle
    skew_om = skew3(th)
    skew_rho = skew3(rho)
    cos_an = np.cos(an)
    sin_an = np.sin(an)

    term1 = 0.5*skew_om
    term2 = (an - sin_an)/an**3
    term3 = skew_om @ skew_rho + skew_rho @ skew_om + skew_om @ skew_rho @ skew_om
    term4 = (1-an**2/2-np.cos(an))/an**4
    term5 = skew_om@skew_om@skew_rho+skew_rho@skew_om@skew_om-3*skew_om@skew_rho@skew_om
    term6 = 0.5*((1-an**2/2-cos_an)/(an**4)-3*(an-sin_an-an**3/6)/(an**5))
    term7 = skew_om@skew_rho@skew_om@skew_om+skew_om@skew_om@skew_rho@skew_om
    Q = term1 + term2*term3 - term4*term5 - term6*term7
    return Q


def left_jacob(vec):
    assert vec.shape == (6,)
    omega = vec[3:6]
    jacob = np.zeros((6,6))
    SO3_left_jacobian = SO3_ops.left_jacob(omega)
    Q = jacob_q_term(vec)
    jacob[0:3,0:3] = SO3_left_jacobian
    jacob[0:3,3:] = Q
    jacob[3:,3:] = SO3_left_jacobian
    return jacob


def inv_left_jacob(vec):
    assert vec.shape == (6,)
    inv_jacob = np.zeros((6,6))
    theta_bold = vec[3:6]
    SO3_inv_left_jacobian = SO3_ops.inv_left_jacob(theta_bold)
    Q = jacob_q_term(vec)
    inv_jacob[0:3,0:3] = SO3_inv_left_jacobian
    inv_jacob[0:3,3:] = -SO3_inv_left_jacobian@Q@SO3_inv_left_jacobian
    inv_jacob[3:,3:] = SO3_inv_left_jacobian
    return inv_jacob

def inv_right_jacob(vec):
    assert vec.shape == (6,)
    return inv_left_jacob(-vec)



