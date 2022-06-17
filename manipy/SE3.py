import numpy as np
from .utils import rodriguez
from .SO3 import SO3
from termcolor import colored

class SE3():
    """A class for operations on transformation matrices

    :param transf_mat_np: A 4 by 4 numpy array
    :type transf_mat_np: np.ndarray
    """
    def __init__(self, transf_mat_np=np.identity(4)):
        assert transf_mat_np.shape == (4,4)
        self.t = transf_mat_np[:3,3]
        self.SO3 = SO3(transf_mat_np[:3,:3])

    @property 
    def mat(self):
        mat = np.identity(4)
        mat[:3, :3] = self.SO3.mat
        mat[:3, 3] = self.t
        return mat

    @staticmethod
    def decompose_Rt(se3_mat):
        assert se3_mat.shape == (4,4)
        R = se3_mat[0:3,0:3]
        t = se3_mat[0:3,3]
        return R, t

    @property
    def R(self):
        return self.SO3.mat

    @staticmethod
    def wedge(vec):
        """SE3 wedge operation

        .. math::
            \\begin{bmatrix} 1 & 2 \\\\ 3 & 4 \end{bmatrix}
        """
        transl = vec[:3]
        rot = vec[3:6]
        lie_alg = np.array([[ 0,      -rot[2], rot[1], transl[0]],
                            [ rot[2],  0,     -rot[0], transl[1]],
                            [-rot[1],  rot[0], 0,      transl[2]],
                            [0,        0,      0,      0        ]])
        return lie_alg

    @staticmethod
    def vee(lie_alg):
        assert lie_alg.shape == (4,4)
        SO3_lie_alg = lie_alg[0:3,0:3]
        rho = lie_alg[0:3,3]
        omega = SO3.vee(SO3_lie_alg)
        return np.concatenate((rho,omega))

    @staticmethod
    def exp(lie_alg):
        assert lie_alg.shape == (4,4)
        transf_mat = np.identity(4)
        u_skew = lie_alg[:3,:3]
        rho = lie_alg[:3, 3]
        R = SO3.exp(u_skew)
        transf_mat[0:3,0:3] = R
        transf_mat[0:3,3] = SO3.left_jacob(theta) @ rho
        return transf_mat

    @staticmethod
    def Exp(vec):
        assert vec.shape == (6,)
        transf_mat = np.identity(4)
        rho = vec[0:3]
        theta = vec[3:6]
        rot_mat = SO3_Exp(theta)
        transf_mat[0:3,0:3] = rot_mat
        transf_mat[0:3,3] = SO3.left_jacob(theta) @ rho
        return transf_mat

    @staticmethod
    def log(se3_mat):
        lie_alg = np.zeros((4,4))
        R = se3_mat[:3,:3]
        t = se3_mat[:3, 3]
        rot_skew = SO3.log(R)
        lie_alg[:3,:3] = rot_skew
        lie_alg[:3, 3] = transl
        return lie_alg

    @staticmethod
    def Log(se3_mat):
        assert se3_mat.shape == (4,4)
        R,t = SE3.decompose_Rt(se3_mat)
        theta = SO3.Log(R)
        rho = SO3.inv_left_jacob(theta) @ t
        rho = np.squeeze(rho)
        return np.concatenate((rho,theta))

    def adjoint(self):
        ad = np.zeros((6,6))
        rot_mat = self.SO3.mat
        skew_t = SO3.wedge(self.t)
        ad[:3,:3] = rot_mat
        ad[:3, 3:6] = skew_t@rot_mat
        ad[3:6, 3:6] = rot_mat
        return ad

    def perturb(self, vec):
        assert vec.shape == (6,)
        perp = SE3.from_vec(vec)
        return self*perp

    def perturb_left(self, vec):
        assert vec.shape == (6,)
        perp = SE3.from_vec(vec)
        return perp*self


    def __mul__(self, other):
        new_T = self.mat@other.mat
        return SE3(new_T)

    def left_jacob():
        pass

    @classmethod
    def from_vec(cls, vec):
        T = cls.Exp(vec)
        return cls(T)

    def as_vec(self):
        return self.Log(self.mat)

    @classmethod
    def from_SE3_np(cls, transf_mat_np):
        return cls(transf_mat_np)

    def as_SE3_np(self):
        return self.mat

    @classmethod
    def from_quat(cls):
        pass

    def as_quat():
        pass

    @classmethod
    def from_euler(cls):
        pass

    def as_euler():
        pass

    @classmethod
    def from_SO3(cls, SO3, t=[.0,.0,.0]):
        t = np.array(t)
        T = np.identity(4)
        T[:3,:3] = SO3.mat
        T[:3,3] = t
        return cls(T)

    def as_SO3(self):
        return self.SO3

    @classmethod
    def from_SO3_np(cls):
        pass

    def as_SO3_np():
        pass

    @classmethod
    def from_rx(cls, angle, t=[.0,.0,.0]):
        rot = SO3.from_rx(angle)
        return cls.from_SO3(rot, t)

    @classmethod
    def from_ry(cls):
        rot = SO3.from_ry(angle)
        return cls.from_SO3(rot, t)

    @classmethod
    def from_rz(cls):
        rot = SO3.from_rz(angle)
        return cls.from_SO3(rot, t)

    @classmethod
    def from_transl(cls, t):
        T = np.identity(4)
        T[:3,3] = t
        return cls(T)

    def inv(self):
        rot = self.SO3.inv()
        transl = -rot.mat@self.t
        return SE3.from_SO3(rot, transl)

    @staticmethod
    def jacob_q_term(vec):
        assert vec.shape == (6,)
        rho = vec[:3] # rho
        th = vec[3:] # theta
        an = np.linalg.norm(th) # angle
        skew_om = skew(th)
        skew_rho = skew(rho)
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

    def left_jacob(self):
        vec = self.Log(self.mat)
        omega = vec[3:6]
        jacob = np.zeros((6,6))
        SO3_left_jacobian = self.SO3.left_jacob(omega)
        Q = self.jacob_q_term(vec)
        
        jacob[0:3,0:3] = SO3_left_jacobian
        jacob[0:3,3:] = Q
        jacob[3:,3:] = SO3_left_jacobian
        return jacob


    def __str__(self):
        mat = self.mat
        rot_col = 'blue'
        t_col = 'green'
        r1 = colored(f'{mat[0,0]:.3f} {mat[0,1]:.3f} {mat[0,2]:.3f}', rot_col)
        r2 = colored(f'{mat[1,0]:.3f} {mat[1,1]:.3f} {mat[1,2]:.3f}', rot_col)
        r3 = colored(f'{mat[2,0]:.3f} {mat[2,1]:.3f} {mat[2,2]:.3f}', rot_col)
        t1 = colored(f'{mat[0,3]:.3f}' , t_col)
        t2 = colored(f'{mat[1,3]:.3f}', t_col)
        t3 = colored(f'{mat[2,3]:.3f}', t_col)
        bot = colored(f'{mat[3,0]:.3f} {mat[3,1]:.3f} {mat[3,2]:.3f} {mat[3,3]:.3f}', 'yellow')
        display_print = f'{r1} {t1} \n{r2} {t2} \n{r3} {t3} \n{bot}\n'
        return display_print

    def __getitem__(self, key):
        return self.mat[key]

    @property
    def shape(self):
        return (4,4)



if __name__ == '__main__':
    vec = np.array([1,2,3,0.2, 0.05, 0.02])
    transf = SE3.from_vec(vec)
    print(transf)
    print(transf[:3,:3])
    print(transf.as_vec())
    """
    display_print = f'{mat[0,0]:.3f} {mat[0,1]:.3f} {mat[0,2]:.3f} {mat[0,3]:.3f} \n' + \
            f'{mat[1,0]:.3f} {mat[1,1]:.3f} {mat[1,2]:.3f} {mat[1,3]:.3f} \n' + \
            f'{mat[2,0]:.3f} {mat[2,1]:.3f} {mat[2,2]:.3f} {mat[2,3]:.3f} \n' + \
            f'{mat[3,0]:.3f} {mat[3,1]:.3f} {mat[3,2]:.3f} {mat[3,3]:.3f} \n'
    """










