import numpy as np
from utils import rodriguez
from SO3 import SO3
from termcolor import colored

class SE3():
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

    @property
    def R(self):
        return self.SO3.mat

    @staticmethod
    def wedge(vec):
        transl = vec[:3]
        rot = vec[3:6]
        lie_alg = np.array([[ 0,      -rot[2], rot[1], transl[0]],
                            [ rot[2],  0,     -rot[0], transl[1]],
                            [-rot[1],  rot[0], 0,      transl[2]],
                            [0,        0,      0,      0        ]])
        return lie_alg

    @staticmethod
    def vee(lie_alg):
        pass

    @staticmethod
    def exp(lie_alg):
        assert lie_alg.shape == (4,4)
        u_skew = lie_alg[:3,:3]
        transl = lie_alg[:3, 3]
        R = SO3.exp(u_skew)
        transf_mat = np.identity(4)
        transf_mat[:3,:3] = R
        transf_mat[:3, 3] = transl
        return transf_mat

    @staticmethod
    def Exp(vec):
        assert vec.shape == (6,)
        transl = vec[:3]
        axang = vec[3:6]
        transf_mat = np.identity(4)
        R = SO3.Exp(axang)
        transf_mat[:3,:3] = R
        transf_mat[:3,3] = transl
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
        R = se3_mat[:3, :3]
        t = se3_mat[:3, 3]
        theta = SO3.Log(R)
        vec = np.hstack((t,theta))
        return vec

    def adjoint():
        pass

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
    def from_SO3(cls):
        pass

    def as_SO3():
        pass

    @classmethod
    def from_SO3_np(cls):
        pass

    def as_SO3_np():
        pass

    @classmethod
    def from_rx(cls):
        pass

    @classmethod
    def from_ry(cls):
        pass

    @classmethod
    def from_rz(cls):
        pass

    @classmethod
    def from_transl(cls):
        pass




    def __str__(self):
        mat = self.mat
        rot_col = 'blue'
        t_col = 'green'
        r1 = colored(f'{mat[0,0]:.3f} {mat[0,1]:.3f} {mat[0,2]:.3f}', rot_col)
        r2 = colored(f'{mat[1,0]:.3f} {mat[1,1]:.3f} {mat[1,2]:.3f}', rot_col)
        r3 = colored(f'{mat[3,0]:.3f} {mat[3,1]:.3f} {mat[3,2]:.3f}', rot_col)
        t1 = colored(f'{mat[0,3]:.3f}' , t_col)
        t2 = colored(f'{mat[1,3]:.3f}', t_col)
        t3 = colored(f'{mat[2,3]:.3f}', t_col)
        bot = colored(f'{mat[3,0]:.3f} {mat[3,1]:.3f} {mat[3,2]:.3f} {mat[3,3]:.3f}', 'yellow')
        display_print = f'{r1} {t1} \n{r2} {t2} \n{r3} {t3} \n{bot}'
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










