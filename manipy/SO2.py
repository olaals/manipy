import numpy as np
from termcolor import colored

class SO2():
    def __init__(self, rot_mat_np=np.identity(2)):
        assert rot_mat_np.shape == (2,2)
        self.mat = rot_mat_np

    def __mul__(self, other):
        new_rot_mat = self.mat@other.mat
        return SO2(new_rot_mat)

    def adjoint(self):
        return 1.0

    def inv(self):
        return SO2(self.mat.transpose())

    @property
    def shape():
        return (2,2)

    @staticmethod
    def Log(rot_mat):
        cosine = rot_mat[0,0]
        sine = rot_mat[1,0]
        angle = np.arctan2(sine,cosine)
        return angle

    @staticmethod
    def Exp(angle):
        cosine = np.cos(angle)
        sine = np.sin(angle)
        rot_mat = np.array([[cosine, -sine],[sine,cosine]])
        return rot_mat

    @classmethod
    def from_angle(cls, angle):
        rot_mat_np = cls.Exp(angle)
        return cls(rot_mat_np)

    def to_angle(self):
        return self.Log(self.mat)

    def __str__(self):
        mat = self.mat
        display_print = f'{mat[0,0]:.3f} {mat[0,1]:.3f} \n' + \
                        f'{mat[1,0]:.3f} {mat[1,1]:.3f} \n'
        display_print = colored(display_print, 'blue')
        return display_print

    @staticmethod
    def wedge(angle):
        angle_wedge = np.array([[.0, -angle], [angle, .0]])
        return angle_wedge


