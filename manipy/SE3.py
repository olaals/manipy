import numpy as np
from .utils import rodriguez
from .SO3 import SO3
from termcolor import colored
from . import SE3_ops

class SE3():
    """A class for operations on transformation matrices

    :param transf_mat_np: A 4 by 4 numpy array
    :type transf_mat_np: np.ndarray
    """
    def __init__(self, transf_mat_np=np.identity(4)):
        assert transf_mat_np.shape == (4,4)
        self.t = transf_mat_np[:3,3]
        self.SO3 = SO3(transf_mat_np[:3,:3])

    @classmethod
    def identity(cls):
        return cls(np.identity(4))

    @property 
    def mat(self):
        mat = np.identity(4)
        mat[:3, :3] = self.SO3.mat
        mat[:3, 3] = self.t
        return mat

    @property
    def R(self):
        return self.SO3.mat

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
        T = SE3_ops.Exp(vec)
        return cls(T)

    def as_vec(self):
        return SE3_ops.Log(self.mat)

    def as_lie_alg(self):
        return SE3_ops.log(self.mat)

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
    def from_ry(cls, angle, t=[.0,.0,.0]):
        rot = SO3.from_ry(angle)
        return cls.from_SO3(rot, t)

    @classmethod
    def from_rz(cls, angle, t=[.0,.0,.0]):
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


    def left_jacob(self):
        vec = SE3_ops.Log(self.mat)
        return SE3_ops.left_jacob(vec)

    def inv_left_jacob(self):
        vec = SE3_ops.Log(self.mat)
        return SE3_ops.inv_left_jacob(vec)

    def right_jacob(self):
        vec = SE3_ops.Log(self.mat)
        return SE3_ops.right_jacob(vec)

    def inv_right_jacob(self):
        vec = SE3_ops.Log(self.mat)
        return SE3_ops.inv_right_jacob(vec)


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










