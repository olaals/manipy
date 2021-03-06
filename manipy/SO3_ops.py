from .utils import *

def wedge(vec):
    vec = np.squeeze(vec)
    assert vec.shape == (3,)
    return skew3(vec)

def vee(lie_alg):
    assert lie_alg.shape == (3,3)
    vec = np.zeros((3))
    vec[0] = lie_alg[2,1]
    vec[1] = lie_alg[0,2]
    vec[2] = lie_alg[1,0]
    return vec

def log(rot_mat):
    cosine = np.clip((np.trace(rot_mat) - 1.0)/2.0, -1.0, 1.0)
    angle = np.arccos(cosine)
    if np.isclose(angle, 0.):
        return rot_mat - np.identity(3)
    lie_alg = (angle/(2*np.sin(angle)))*(rot_mat-rot_mat.transpose())
    return lie_alg

def Log(rot_mat):
    assert rot_mat.shape == (3,3)
    cos_theta = (np.trace(rot_mat) - 1) / 2
    cos_upper_lim = 1.0
    cos_lower_lim = -1.0
    cos_theta = np.clip(cos_theta, cos_lower_lim, cos_upper_lim)
    theta = np.arccos(cos_theta)
    if np.isclose(theta, 0.0):
        return vee(rot_mat - np.eye(3))
    return (theta / (2 * np.sin(theta))) * vee(rot_mat - rot_mat.T)


def exp(vec):
    assert lie_alg.shape == (3,3)
    return Exp(vee(vec))

def Exp(vec):
    vec = np.squeeze(vec)
    assert vec.shape == (3,)
    angle = np.linalg.norm(vec)
    if np.isclose(angle, 0.):
        return np.identity(3)+wedge(vec)
    axis = vec/angle
    return rodriguez(wedge(axis), angle)

def adjoint(rot_mat):
    return rot_mat

def left_jacob(vec):
    theta = np.linalg.norm(vec)
    if np.isclose(theta, 0.0):
        return np.eye(3) + 0.5 * wedge(vec)
    theta_bold_skew = skew3(vec)
    term1 = ((theta - np.sin(theta)) / theta**3) * theta_bold_skew @ theta_bold_skew
    term2 = np.eye(3) + ((1 - np.cos(theta)) / theta**2) * theta_bold_skew
    return term1 + term2


def right_jacob(vec):
    left_jacob = left_jacob(vec)
    right_jacob = left_jacob.T
    return right_jacob


def inv_left_jacob(vec):
    theta = np.linalg.norm(vec)
    if np.isclose(theta, 0.0):
        return np.eye(3) - 0.5 * wedge(vec)
    theta_bold_skew = skew3(vec)
    fraq_1 = 1 / theta**2
    fraq_2 = (1 + np.cos(theta)) / (2 * theta * np.sin(theta))
    term1 = np.eye(3) - 0.5 * theta_bold_skew
    term2 = (fraq_1 - fraq_2) * theta_bold_skew @ theta_bold_skew
    return  term1 + term2

def inv_right_jacob(vec):
    inv_left_jacob = inv_left_jacob(vec)
    inv_right_jacob = inv_left_jacob.T
    return inv_right_jacob




