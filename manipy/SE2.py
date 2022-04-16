import numpy as np
from .SO2 import SO2
from termcolor import colored

class SE2():
    def __init__(self, transf_mat_np=np.identity(3)):
        assert transf_mat_np.shape == (3,3)
        rot_mat = transf_mat_np[:2,:2]
        transl = transf_mat_np[:2, 2]
        self.SO2 = SO2(rot_mat)
        self.t = transl

    @property
    def mat(self):
        mat = np.identity(3)
        mat[:2,:2] = self.SO2.mat
        mat[:2,2] = self.t
        return mat

    @property
    def R(self):
        return self.SO2.mat

    @staticmethod
    def Exp(vec):
        assert vec.shape == (3,)
        transf_mat = np.identity(3)
        angle = vec[2]
        R = SO2.Exp(angle)
        transl = vec[:2]
        transf_mat[:2,:2] = R
        transf_mat[:2, 2] = transl
        return transf_mat

    @staticmethod
    def Log(transf_mat):
        R = transf_mat[:2,:2]
        angle = SO2.Log(R)
        transl = transf_mat[:2,2]
        vec = np.hstack((transl, angle))
        return vec

    @classmethod
    def from_vec(cls, vec):
        transf_mat = cls.Exp(vec)
        return cls(transf_mat)

    def as_vec(self):
        return self.Log(self.mat)

    def adjoint(self):
        ad = np.identity(3)
        ad[:2,:2] = self.SO2.mat
        ad[:2,2] = SO2.wedge(1.0)@self.t
        return ad

    def __str__(self):
        mat = self.mat
        rot_col = 'blue'
        t_col = 'green'
        r1 = colored(f'{mat[0,0]:.3f} {mat[0,1]:.3f} ', rot_col)
        r2 = colored(f'{mat[1,0]:.3f} {mat[1,1]:.3f} ', rot_col)
        t1 = colored(f'{mat[0,2]:.3f}', t_col)
        t2 = colored(f'{mat[1,2]:.3f}', t_col)
        bot = colored(f'{mat[2,0]:.3f} {mat[2,1]:.3f} {mat[2,2]:.3f}', 'yellow')
        display_print = f'{r1} {t1} \n{r2} {t2} \n{bot}\n'
        return display_print



        











