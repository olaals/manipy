from manipy import SE3_ops, SE3
import math
import numpy as np
import time


def test_right_jac():
    pass


def test_inv_right_jac():
    SE3_rx_vec = SE3.from_rx(math.pi).as_vec()
    print("vec")
    print(SE3_rx_vec)
    SE3_rx = SE3.from_vec(SE3_rx_vec)
    print("T")
    print(SE3_rx)

    y_pert = np.zeros(6)
    y_pert[4] = -0.005
    print("Perturb vector")
    print(y_pert)
    print("perturb from y comp")

    print("Jacobian")

    vec_from_pert = SE3_rx.perturb(y_pert).as_vec()
    inv_right_jac = SE3_rx.inv_right_jacob()
    print("inv right jac")
    print(inv_right_jac)
    new_comp = inv_right_jac@y_pert
    print("change of reference y_pert")
    print(new_comp)
    vec_from_jac = SE3_rx_vec+new_comp
    print("Vec from pert")
    print(vec_from_pert)
    print("Vec from jac")
    print(vec_from_jac)
    assert np.allclose(vec_from_pert, vec_from_jac)








if __name__ == '__main__':
    test_inv_right_jac()



