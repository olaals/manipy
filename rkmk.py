from manipy import SE3_ops, SE3
import numpy as np
import math
import matplotlib.pyplot as plt





def constant_velocity(_, __):
    vec = SE3.from_rz(0.1*math.pi, t=[0.1,0,0]).as_vec()
    return vec

def time_var_velocity(_, t):
    vec = SE3.from_rz(0.1+0.5*np.sin(t), [0.1,0,0]).as_vec()
    return vec

def time_and_space_var_velocity(T, t):
    y_loc = T[1,3]
    vec = SE3.from_rz(0.1*math.pi, t=[0.1*np.sin(t)+y_loc,0,0]).as_vec()
    return vec


def RKMK_euler(steps, kinematics, sim_duration):
    h = sim_duration/steps

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


def RKMK2(steps, kinematics, sim_duration):
    h = sim_duration/steps


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
        print("RKMK2 F1")
        print(F1)

        c2=1.0
        a2_1 = 1.0
        Theta2 = a2_1*F1
        print("RKMK2 Theta2", Theta2)
        theta2 = h*kinematics(T*SE3.from_vec(Theta2), time+c2*h)
        print("RKMK2 theta2", theta2)
        inv_jac = SE3_ops.inv_right_jacob(Theta2)
        #print(inv_jac)
        #inv_jac = np.identity(6)
        F2 = inv_jac@theta2
        print("RKMK2 F2")
        print(F2)
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
    
def solve_RKMK_row(A_row, Fs):
    Theta = np.zeros(6)
    for j in range(len(Fs)):
        print("solve RKMK row", j)
        print("A_row", A_row[j])
        Theta+=Fs[j]*A_row[j]
        print(Theta)
    return Theta

def sum_to_current_est(Fs, Bs):
    assert (len(Fs) == len(Bs))
    sum_vec = np.zeros(6)
    for i in range(len(Fs)):
        sum_vec += Bs[i]*Fs[i]
    return sum_vec


def RKMK(steps, kinematics, sim_duration, butcher_tableu):
    assert isinstance(butcher_tableu, tuple)
    As,Bs,Cs = butcher_tableu
    A_rows, A_cols = As.shape
    assert A_rows == len(Cs)
    assert A_cols == len(Bs)
    xs = []
    ys = []

    stages = len(Cs)
    print("Stages", stages)
    time = 0
    h = sim_duration/steps
    T0 = SE3.identity()

    T = T0
    for step in range(0,steps):
        print("Step", step)
        Fs = []
        Theta0 = np.zeros(6)
        F0 = h*kinematics(T0, time+Cs[0]*h)
        print(F0)
        Fs.append(F0)
        for s in range(1,stages):
            print("Stage", s)
            A_row = As[s,:]
            Theta = solve_RKMK_row(A_row, Fs)
            print("Theta g", Theta)
            theta = h*kinematics(T.perturb(Theta), time+Cs[s]*h)
            print("theta g", theta)
            inv_jac = SE3_ops.inv_right_jacob(Theta)
            F = inv_jac@theta
            print(F)
            Fs.append(F)
        Theta = sum_to_current_est(Fs, Bs)
        T = T.perturb(Theta)
        xs.append(T.t[0])
        ys.append(T.t[1])
        time += h
    print("xs", xs)
    print("xs", ys)
    plt.plot(xs,ys)
            



        






def RKMK4(steps, kinematics, sim_duration):
    h = sim_duration/steps


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

def get_heun_butcher():
    As = np.array([[0.0,0.0],[1.0,0.0]])
    Bs = np.array([0.5,0.5])
    Cs = np.array([0.0,1.0])
    heuns = (As,Bs,Cs)
    return heuns

def get_euler_butcher():
    As = np.array([[0.0]])
    Bs = np.array([1.0])
    Cs = np.array([0.0])
    return (As,Bs,Cs)

def get_rk4_butcher():
    As = np.zeros((4,4))
    As[1,0] = 0.5
    As[2,1] = 0.5
    As[3,2] = 1.0
    print(As)
    Bs = np.array([1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0])
    Cs = np.array([0.0, 0.5,0.5,1.0])
    return (As,Bs,Cs)







if __name__ == '__main__':
    SIM_DUR = 20
    #kinematics = constant_velocity
    kinematics = time_var_velocity
    #kinematics = time_and_space_var_velocity
    #butch_tab = get_euler_butcher()
    #butch_tab = get_heun_butcher()
    butch_tab = get_rk4_butcher()

    RKMK(20, kinematics, SIM_DUR, butch_tab)
    #RKMK2(20, kinematics, SIM_DUR)
    #RKMK_euler(20, kinematics, SIM_DUR)
    #RKMK4(10, kinematics, SIM_DUR)

    plt.show()



    
    







