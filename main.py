
import numpy as np
import time
import matplotlib.pyplot as plt
import control


from dynamics.DoubleInvertedPendulum import DoubleInvertedPendulumDynamics
from dynamics.StateSpace import StateSpace

def run():
    L1 = 2.0
    L2 = 2.0
    f1 = 0.0
    f2 = 0.0

    dt = 0.02
    dead_time = 0.0
    add_measurement_noise = False

    controller_dt = 0.02

    dynamics = DoubleInvertedPendulumDynamics(
        L1 = L1,
        L2 = L2,
        l1 = 1.0,
        l2 = 1.0,
        M1 = 1.0,
        M2 = 1.0,
        I1 = 1.0/ 3,
        I2 = 1.0/ 3,
        c1 = 0.3,
        c2 = 0.3,
        f1=f1,
        f2=f2,
        use_linearized_dynamics = True,
    )

    desired_state = np.radians([0.0, 0.0, 0.0, 0.0]) # theta1, theta1_dot, theta2, theta2_dot
    state_space = StateSpace(dynamics)

    check_stability = True
    if check_stability :
        W_c = control.ctrb(state_space.A, state_space.B)
        if np.linalg.matrix_rank(W_c) != state_space.A.shape[0]:
            print("system is not controllable !\n",flush=True)
        else:
            print("System Controllable !",flush=True)

    sys_ss = control.ss(state_space.A, state_space.B, state_space.C, state_space.D)


if __name__ == "__main__":
    run()