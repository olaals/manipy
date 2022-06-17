from .utils import *
import sys
from termcolor import colored
from . import SO3_ops

class SO3():
    def __init__(self, rot_mat_np=np.identity(3)):
        assert rot_mat_np.shape == (3,3)
        self.mat = rot_mat_np

    def __mul__(self, other):
        new_rot_mat = self.mat@other.mat
        return SO3(new_rot_mat)

    def adjoint(self):
        return self.mat

    @classmethod
    def from_vec(cls, vec):
        return cls(cls.Exp(vec))

    def as_vec(self):
        return SO3_ops.Log(self.mat)

    def as_logarithm(self):
        return SO3_ops.log(self.mat)

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

    @classmethod
    def from_rx(cls, angle):
        vec = np.array([angle,0,0])
        SO3_mat = SO3_ops.Exp(vec)
        return cls(SO3_mat)

    @classmethod
    def from_ry(cls, angle):
        vec = np.array([0,angle,0])
        SO3_mat = SO3_ops.Exp(vec)
        return cls(SO3_mat)

    @classmethod
    def from_rz(cls, angle):
        vec = np.array([0,0,angle])
        SO3_mat = SO3_ops.Exp(vec)
        return cls(SO3_mat)

    def __getitem__(self, key):
        return self.mat[key]

    def left_jacob(self):
        vec = self.as_vec()
        return SO3_ops.left_jacob(vec)

    def right_jacob(self):
        vec = self.as_vec()
        return SO3_ops.right_jacob(vec)

    def inv_left_jacob(self):
        vec = self.as_vec()
        return SO3_ops.inv_left_jacob(vec)

    def inv_right_jacob(self):
        vec = self.as_vec()
        return SO3_ops.inv_right_jacob(vec)

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



