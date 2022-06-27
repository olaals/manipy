from manipy import SE3_ops, SE3
import numpy as np
import math
import matplotlib.pyplot as plt


def constant_velocity(_, __):
    vec = SE3.from_rz(0.01*math.pi, t=[0.01,0.0,0.0]).as_vec()
    return vec

def time_var_velocity(_, t):
    vec = SE3.from_rz(0.01+0.05*np.sin(t/15.0), [0.05,0,0]).as_vec()
    return vec

def time_and_space_var_velocity(T, t):
    y_loc = T[1,3]
    vec = SE3.from_rz(0.1*math.pi, t=[0.1*np.sin(t)+y_loc,0,0]).as_vec()
    return vec

    
def solve_RKMK_row(A_row, Fs):
    Theta = np.zeros(6)
    for j in range(len(Fs)):
        Theta+=Fs[j]*A_row[j]
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
    print("Time step", h)
    T0 = SE3.identity()

    T = T0
    xs.append(T.t[0])
    ys.append(T.t[1])
    for step in range(0,steps):
        print("Step", step)
        Fs = []
        Theta0 = np.zeros(6)
        F0 = h*kinematics(T0, time+Cs[0]*h)
        Fs.append(F0)
        for s in range(1,stages):
            print("Stage", s)
            A_row = As[s,:]
            Theta = solve_RKMK_row(A_row, Fs)
            theta = h*kinematics(T.perturb(Theta), time+Cs[s]*h)
            inv_jac = SE3_ops.inv_right_jacob(Theta)
            F = inv_jac@theta
            Fs.append(F)
        Theta = sum_to_current_est(Fs, Bs)
        T = T.perturb(Theta)
        xs.append(T.t[0])
        ys.append(T.t[1])
        time += h
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


def plot_time_varying():
    SIM_DUR = 100
    kinematics = time_var_velocity
    rk1_btab = get_euler_butcher()
    rk2_btab = get_heun_butcher()
    rk4_btab = get_rk4_butcher()

    steps = 12
    gt_steps = 1000

    RKMK(gt_steps, kinematics, SIM_DUR, rk1_btab)
    RKMK(steps, kinematics, SIM_DUR, rk1_btab)
    RKMK(steps, kinematics, SIM_DUR, rk2_btab)
    RKMK(steps, kinematics, SIM_DUR, rk4_btab)

    # plotting
    labels = [
        f'RKMK4 {gt_steps} steps',
        f'RKMK1 {steps} steps',
        f'RKMK2 {steps} steps',
        f'RKMK4 {steps} steps',
    ]
    plt.legend(labels)
    plt.title('Time varying velocity')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.show()

def plot_constant_vel():
    SIM_DUR = 100
    kinematics = constant_velocity
    rk1_btab = get_euler_butcher()
    rk2_btab = get_heun_butcher()
    rk4_btab = get_rk4_butcher()

    steps = 10
    gt_steps = 100

    RKMK(gt_steps, kinematics, SIM_DUR, rk1_btab)
    RKMK(steps, kinematics, SIM_DUR, rk1_btab)
    RKMK(steps, kinematics, SIM_DUR, rk2_btab)
    RKMK(steps, kinematics, SIM_DUR, rk4_btab)

    # plotting
    labels = [
        f'RKMK4 {gt_steps} steps',
        f'RKMK1 {steps} steps',
        f'RKMK2 {steps} steps',
        f'RKMK4 {steps} steps',
    ]
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(labels)
    plt.title('Constant velocity')

    plt.show()







if __name__ == '__main__':
    plot_constant_vel()
    plot_time_varying()



    
    







