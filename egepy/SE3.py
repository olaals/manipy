import numpy as np
from utils import rodriguez


class SE3():
    def __init__(self):
        self.dof = 6
        self.dim = 4
        self.mat = np.identity(4)

    @staticmethod
    def __wedge(vec):
        transl = vec[:3]
        rot = vec[3:6]
        lie_alg = np.array([[ 0,      -rot[2], rot[1], transl[0]],
                            [ rot[2],  0,     -rot[0], transl[1]],
                            [-rot[1],  rot[0], 0,      transl[2]],
                            [0,        0,      0,      0        ]])
        return lie_alg

    @staticmethod
    def __vee(lie_alg):
        pass

    @staticmethod
    def __exp(lie_alg):
        u_skew = lie_alg[:3,:3]
        transl = lie_alg[:3, 3]
        rot_mat = rodriguez(u_skew)
        pass

    @staticmethod
    def __log(se3_mat):
        pass

    def adjoint():
        pass

    def left_jacob():
        pass

    @classmethod
    def from_vec(cls, vec):
        lie_alg = cls.__wedge(vec)
        print(lie_alg)

    def as_vec():
        pass

    @classmethod
    def from_SE3_np(cls):
        pass

    def as_SE3_np():
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
        display_print = f'{mat[0,0]} {mat[0,1]} {mat[0,2]} {mat[0,3]} \n' + \
                        f'{mat[1,0]} {mat[1,1]} {mat[1,2]} {mat[1,3]} \n' + \
                        f'{mat[2,0]} {mat[2,1]} {mat[2,2]} {mat[2,3]} \n' + \
                        f'{mat[3,0]} {mat[3,1]} {mat[3,2]} {mat[3,3]} \n'
        return display_print

    def __getitem__(self, key):
        return self.mat[key]

    @property
    def shape(self):
        return (4,4)



if __name__ == '__main__':
    vec = np.array([1,2,3,0.1, 0.2, 0.3])


    s = SE3.from_vec(vec)

    s2 = SE3()
    s2 = s2.from_vec(vec)





