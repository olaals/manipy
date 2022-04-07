import numpy as np

def skew3(vec):
    skew = np.array([[0,      -vec[2],  vec[1]],
                    [ vec[2], 0,       -vec[0]],
                    [-vec[1], vec[0],  0      ]])
    return skew
    

def rodriguez(u_skew):
    assert u.shape == (3,3)
    theta = 
    u_skew = 
    rot_mat =  np.identity(3) + u_skew*np.sin(theta) + u_skew@u_skew*(1-np.cos(theta))
    return rot_mat


