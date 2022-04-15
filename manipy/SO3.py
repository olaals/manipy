from .utils import *
import sys
from termcolor import colored

class SO3():
    def __init__(self, rot_mat_np=np.identity(3)):
        assert rot_mat_np.shape == (3,3)
        self.mat = rot_mat_np

    def __mul__(self, other):
        new_rot_mat = self.mat@other.mat
        return SO3(new_rot_mat)

    @staticmethod
    def wedge(vec):
        vec = np.squeeze(vec)
        assert vec.shape == (3,)
        return skew3(vec)

    @staticmethod
    def vee(lie_alg):
        assert lie_alg.shape == (3,3)
        vec = np.zeros((3))
        vec[0] = lie_alg[2,1]
        vec[1] = lie_alg[0,2]
        vec[2] = lie_alg[1,0]
        return vec

    @staticmethod
    def exp(lie_alg):
        assert lie_alg.shape == (3,3)
        return SO3.Exp(SO3.vee(vec))

    @staticmethod
    def Exp(vec):
        vec = np.squeeze(vec)
        assert vec.shape == (3,)
        angle = np.linalg.norm(vec)
        if np.isclose(angle, 0.):
            return np.identity(3)+SO3.wedge(vec)
        axis = vec/angle
        return rodriguez(SO3.wedge(axis), angle)


    @staticmethod
    def log(rot_mat):
        cosine = np.clip((np.trace(rot_mat) - 1.0)/2.0, -1.0, 1.0)
        angle = np.arccos(cosine)
        if np.isclose(angle, 0.):
            return rot_mat - np.identity(3)
        lie_alg = (angle/(2*np.sin(angle)))*(rot_mat-rot_mat.transpose())
        return lie_alg
    
    @staticmethod
    def Log(rot_mat):
        lie_alg = SO3.log(rot_mat)
        return SO3.vee(lie_alg)

    def adjoint(self):
        return self.mat

    @classmethod
    def from_vec(cls, vec):
        return cls(cls.Exp(vec))

    def as_vec(self):
        return SO3.Log(self.mat)

    @classmethod
    def from_np(cls, rot_mat_np):
        return cls(rot_mat_np)

    def to_np(self):
        return self.mat

    def inv(self):
        return SO3(self.mat.transpose())

    def from_quat():
        pass

    def to_quat():
        pass

    def from_euler():
        pass

    def to_euler():
        pass

    def __getitem__(self, key):
        return self.mat[key]

    @property
    def shape(self):
        return (3,3)

    def __str__(self):
        mat = self.mat
        display_print = f'{mat[0,0]:.3f} {mat[0,1]:.3f} {mat[0,2]:.3f} \n' + \
                        f'{mat[1,0]:.3f} {mat[1,1]:.3f} {mat[1,2]:.3f} \n' + \
                        f'{mat[2,0]:.3f} {mat[2,1]:.3f} {mat[2,2]:.3f} \n'
        display_print = colored(display_print, 'blue')
        return display_print










if __name__ == '__main__':
    vec = np.array([0.2, 0.3, 0.5])
    R = SO3.from_vec(vec)
    v = SO3.Log(R.mat)
    print(v)



