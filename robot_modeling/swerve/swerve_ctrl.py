
import os

import numpy as np
import matplotlib.pyplot as plt

from control import ss, tf, LinearIOSystem, NonlinearIOSystem, summing_junction, interconnect, input_output_response
from lmfit import Parameters

from common.nonlinear_systems import second_order_sys_x2_deadband
from common.control_utils import clamp, clamp_update

# Module name used by main.py as a user argument.
SWERVE_MODULE_CTRL = "swerve_ctrl"

# Swerve module dynamics
SWERVE_MODEL = second_order_sys_x2_deadband

# Swerve module constants
params = {
    'b': -10.918190004330565,
    'c': 16989.456476336083,
    'd': 0.10173287784712799
}

def swerve_module_output(t, x, u, params):
    return x

def closed_loop_swerve(pid_gains):
    # Build the system from the parameters
    A = np.array([[0, 1], [0, params['b']]])
    B = np.array([0, params['c']])
    C = np.array([1])
    D = np.array([0, 0])

    # Swerve state space
    # swerve_ss = ss(A,B,C,D)

    # Swerve Nonlinear IO System
    swerve_sys = NonlinearIOSystem(SWERVE_MODEL, swerve_module_output, inputs=('u_clamp'), outputs=('x', 'x_dot'), states=('x', 'x_dot'), name="swerve", params=params)

    # Derivative block (for sampling angular velocity)
    # d_filter = 0.001
    # x_dot_tf = tf([1, 0],[d_filter, 1])
    # x_dot_sys = LinearIOSystem(x_dot_tf, inputs=('x'), outputs=("x_dot"), name="x_dot_sys")

    clamp_sys = NonlinearIOSystem(clamp_update, clamp, inputs="u", outputs="u_clamp", states=1, name="clamp_fn")

    # PID Gains
    Kp = pid_gains['p']
    Ki = pid_gains['i']
    Kd = pid_gains['d']

    # PID control
    pid_A = np.array([0])
    pid_B = np.array([Ki, 0])
    pid_C = np.array([1])
    pid_D = np.array([Kp, Kd])
    pid_ss = ss(pid_A, pid_B, pid_C, pid_D)

    # PID Linear IO System
    pid_sys = LinearIOSystem(pid_ss, inputs=('x_error','x_dot'), outputs=('u'), states=('integrator'), name='pid')

    # Summing block for error
    error_sum = summing_junction(inputs=['x_goal', '-x'], output='x_error', name="x_error_sum")

    # Interconnect the system
    cl_sys = interconnect((pid_sys, clamp_sys, swerve_sys, error_sum), 
        inplist=['x_goal'],
        outlist=['x', 'u_clamp', 'x_dot'],
        name="closed_loop")

    return cl_sys

def swerve_control():
    
    # PID Gains
    pid_gains = {
        'p': 1.5,
        'i': 0.01,
        'd': -0.1
    }

    # Get the closed loop system
    cl_sys = closed_loop_swerve(pid_gains)

    # Simulate the controlled system
    # Simulation timing
    Tf = 0.5 #sec
    dt = 0.01
    n_times = round(Tf / dt)
    t_sim = np.linspace(0, Tf, n_times)

    # Initial condtions
    x0 = np.zeros((4,1))

    # Angle target
    x_setpoint = -90

    # Control signal
    x_U = x_setpoint * np.ones((1, n_times))

    # Combine inputs
    U = x_U

    # Simulate
    t, yout = input_output_response(cl_sys, t_sim, U, x0, params=params)

    # Setpoint line
    y_setpoint = x_setpoint * np.ones(np.size(t))

    # Plot results
    fig, axs = plt.subplots(3)
    axs[0].plot(t, yout[0,:])
    axs[0].plot(t, y_setpoint)
    axs[0].set_ylabel("Angle (deg)")

    axs[1].plot(t, yout[1,:])
    # axs[1].plot(t, yout[2,:])
    axs[1].set_xlabel("Time (sec)")
    axs[1].set_ylabel("Control Power")

    axs[2].plot(t, yout[2,:])
    # axs[1].plot(t, yout[2,:])
    axs[2].set_xlabel("Time (sec)")
    axs[2].set_ylabel("Angular Rate (deg/s)")

    plt.suptitle("Swerve Module")

    # Pause after plotting all results
    plt.show(block=False)
    plt.pause(0.001)  # Pause
    input("hit[enter] to end.")
    plt.close('all')



def plot_results(t, real_traj, sys_traj, actuator):
    x = real_traj[:, 0]
    x_rate = real_traj[:, 1]

    # Plot the results for the time slice
    # x
    plt.figure()
    plt.plot(t, x)
    plt.plot(t, sys_traj[0, :])

    plt.title("x")
    plt.xlabel("Time (sec)")
    plt.ylabel("x")
    plt.legend(("Real", "SysID"))

    # x Rate
    plt.figure()
    plt.plot(t, x_rate)
    plt.plot(t, sys_traj[1, :])

    plt.title("x rate")
    plt.xlabel("Time (sec)")
    plt.ylabel("x rate (rad/s)")
    plt.legend(("Real", "SysID"))

    # Actuators
    plt.figure()
    plt.plot(t, actuator)

    plt.title("Control Input")
    plt.xlabel("Time (sec)")
    plt.ylabel("Power (%)")

# Run the swerve system ID if this script is run directly
if __name__ == "__main__":
    swerve_sysid()