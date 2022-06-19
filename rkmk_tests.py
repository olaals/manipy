from manipy import SE3_ops, SE3
import numpy as np
import math
import matplotlib.pyplot as plt

def constant_velocity(_, __):
    vec = SE3.from_rz(math.pi/1000.0, t=[0.1,0,0]).as_vec()
    return vec

def time_var_velocity(_, t):
    vec = SE3.from_rz(math.pi, t=[0.1*np.sin(t),0,0]).as_vec()
    return vec

def time_and_space_var_velocity(T, t):
    y_loc = T[1,3]
    vec = SE3.from_rz(0.5*math.pi, t=[0.1*np.sin(t)+y_loc,0,0]).as_vec()
    return vec


def RKMK_euler(steps, kinematics):
    h = 1.5/steps

    T0 = SE3.identity()
    Ts = []
    Ts.append(T0)

    T=T0
    xs = []
    ys = []
    xs.append(T0.t[0])
    ys.append(T0.t[1])

    time = 0

    for i in range(steps):
        c=0
        Theta1 = np.zeros(6)
        theta = h*kinematics(T*SE3.from_vec(Theta1), time+c*h)
        F = SE3_ops.inv_right_jacob(Theta1)@theta
        #F = np.identity(6)@theta
        Theta = 1.0*F
        T = T.perturb(Theta)
        xs.append(T.t[0])
        ys.append(T.t[1])
        time += h

    print(T)
    plt.plot(xs,ys)


def RKMK2(steps, kinematics):
    h = 1.5/steps


    T0 = SE3.identity()
    Ts = []
    Ts.append(T0)

    T=T0
    xs = []
    ys = []
    xs.append(T0.t[0])
    ys.append(T0.t[1])
    time = 0

    for i in range(steps):
        c1=0
        Theta1 = np.zeros(6)
        theta1 = h*kinematics(T*SE3.from_vec(Theta1), time+c1*h)
        F1 = SE3_ops.inv_right_jacob(Theta1)@theta1

        c2=1.0
        a2_1 = 0.5
        Theta2 = a2_1*F1
        theta2 = h*kinematics(T*SE3.from_vec(Theta2), time+c2*h)
        inv_jac = SE3_ops.inv_right_jacob(Theta2)
        #print(inv_jac)
        #inv_jac = np.identity(6)
        F2 = inv_jac@theta2
        """
        print("F1")
        print(F1)
        print("F2")
        print(F2)
        """

        Theta = 0.5*F1+0.5*F2
        T = T.perturb(Theta)

        xs.append(T.t[0])
        ys.append(T.t[1])
        time += h


    print(T)
    plt.plot(xs,ys)



def RKMK4(steps, kinematics):
    h = 1.5/steps


    T0 = SE3.identity()
    Ts = []
    Ts.append(T0)

    T=T0
    xs = []
    ys = []
    xs.append(T0.t[0])
    ys.append(T0.t[1])
    time = 0

    for i in range(steps):
        c1=0
        Theta1 = np.zeros(6)
        theta1 = h*kinematics(T*SE3.from_vec(Theta1), time+c1*h)
        F1 = SE3_ops.inv_right_jacob(Theta1)@theta1

        c2=0.5
        a2_1 = 0.5
        Theta2 = a2_1*F1
        theta2 = h*kinematics(T*SE3.from_vec(Theta2), time+c2*h)
        inv_jac = SE3_ops.inv_right_jacob(Theta2)
        #print(inv_jac)
        #inv_jac = np.identity(6)
        F2 = inv_jac@theta2
        """
        print("F1")
        print(F1)
        print("F2")
        print(F2)
        """

        c3 = 0.5
        a3_2 = 0.5
        Theta3 = a3_2*F2
        theta3 = h*kinematics(T*SE3.from_vec(Theta3), time+c3*h)
        inv_jac = SE3_ops.inv_right_jacob(Theta3)
        F3 = inv_jac@theta3

        c4 = 1
        a4_3 = 1
        Theta4 = a4_3*F3
        theta4 = h*kinematics(T*SE3.from_vec(Theta4), time+c4*h)
        inv_jac = SE3_ops.inv_right_jacob(Theta4)
        F4 = inv_jac@theta4

        Theta = (1/6)*F1+(1/3)*F2+(1/3)*F3+(1/6)*F4
        T = T.perturb(Theta)

        xs.append(T.t[0])
        ys.append(T.t[1])
        time += h


    print(T)
    plt.plot(xs,ys)


if __name__ == '__main__':
    kinematics = constant_velocity
    #kinematics = time_var_velocity
    #kinematics = time_and_space_var_velocity

    RKMK1_steps = 6
    RKMK2_steps = 4
    RKMK4_steps = 4
    RKMK4_steps2 = 100
    RKMK_euler(RKMK1_steps, kinematics)
    RKMK2(RKMK2_steps, kinematics)
    RKMK4(RKMK4_steps, kinematics)
    RKMK4(RKMK4_steps2, kinematics)
    labels = [
        f'RKMK1 {RKMK1_steps} steps',
        f'RKMK2 {RKMK2_steps} steps',
        f'RKMK4 {RKMK4_steps} steps',
        f'RKMK4 {RKMK4_steps2} steps'
    ]
    plt.legend(labels)
    plt.show()
    
    







