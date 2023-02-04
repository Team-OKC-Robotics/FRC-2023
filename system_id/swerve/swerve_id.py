
import os

import numpy as np
import matplotlib.pyplot as plt

from scipy.signal import savgol_filter
from lmfit import Parameters, minimize

from common.linear_systems import second_order_integrator_x2, simulate_system
from common.cost_functions import mean_squared_error
from common.load_frc_logs import load_sysid_logs

# Module name used by main.py as a user argument.
SWERVE_MODULE = "swerve"
# DEFAULT_SWERVE_FILE = "FRC_20230203_002759"
# SWERVE_T0 = 10.9
# SWERVE_TF = 11.8
DEFAULT_SWERVE_FILE = "FRC_20230203_015512"
# SWERVE_T0 = 35.5
# SWERVE_TF = 38.5
SWERVE_T0 = 22.5
SWERVE_TF = 23.6
SWERVE_MODEL = second_order_integrator_x2

# INIT_TIMES  = [11.8, 22.55, 23.25]
# FINAL_TIMES = [13.0, 22.9,   23.6]

INIT_TIMES  = [22.5]
FINAL_TIMES = [23.6]

def unwrap_abs_encoder(x):
    """
    The swerve encoder is absolute and reports measurements between 0 and 360. 
    This adds a discontinuity that messes up system ID (because we numerically 
    compute the angular velocity x_rate). This function serves to "unwrap" the 
    absolute encoder measurement so we can get a continuous x
    """
    x_unwrapped = [x[0]]
    x_int = 0

    for i in range(1, len(x)):
        delta = x[i] - x[i-1]

        # If we didn't wrap around, then simply add to the integrator.
        if abs(delta) < 20:
            x_int += delta
        else:
            # Otherwise we just wrapped around. Compute the true delta.
            if x[i] > x[i-1]:
                # We wrapped to +360, so compute a negative turn across 360 as the change
                delta = (x[i] - 360) - x[i-1]
                x_int += delta
            else:
                # We wrapped to 0, so compute a positive turn across 0 as the change
                delta = x[i] + (360 - x[i-1])
                x_int += delta
            
        x_unwrapped.append(x_int)
        
    return np.array(x_unwrapped)



def load_swerve_data(data_file):
    # Load data
    # Note: The '"/module/variable"' quote format is really important for parsing the file.
    sysid_dict = {
        '"/swerve/steer_enc"': [[], []],
        '"/swerve/output"':  [[], []]
    }
    telemetry_dict = load_sysid_logs(data_file, sysid_dict)


    # Break data into its component parts
    t = np.array(telemetry_dict['"/swerve/steer_enc"'][0])
    x = np.array(telemetry_dict['"/swerve/steer_enc"'][1])
    x = unwrap_abs_encoder(x)
    x_rate = np.gradient(x, t)
    # x_rate = savgol_filter(x_rate, 5, 2)
    actuator = np.array(telemetry_dict['"/swerve/output"'][1])
    ref_traj = np.vstack((x, x_rate))
    ref_traj = ref_traj.T

    data_pkg = {
        't': t,
        'x': x,
        'x_rate': x_rate,
        'actuator': actuator,
        'ref_traj': ref_traj
    }

    return data_pkg

def slice_data(data, t0, tf):
    # Get the time slicer
    t = data['t']
    t_slice = (t <= tf) & (t >= t0)

    # Slice data
    s_data = dict()
    keys = ['t', 'x', 'x_rate', 'actuator', 'ref_traj']
    for k in keys:
        if k == 'ref_traj':
            s_data[k] = data[k][t_slice, :]
        else:
            s_data[k] = data[k][t_slice]

    return s_data
    



def swerve_sysid(file=DEFAULT_SWERVE_FILE, t0=SWERVE_T0, tf=SWERVE_TF):
    # Get this script's actual directory so the relative pathing doesn't get rekt.
    module_dir = os.path.dirname(os.path.realpath(__file__))

    # Load the swerve data
    full_data = load_swerve_data(f"{module_dir}/data/{file}.csv")

    # Cut off the first second of the full data since it is usually bad
    full_data = slice_data(full_data, 1, full_data['t'][-1])

    # Do the system ID (find the best parameters for the system)
    params = Parameters()
    # params.add('a', value=-0.5, min=-200.0, max=-0.02)
    params.add('b', value=-2.0, min=-30.0, max=-0.01)
    params.add('c', value=15000.0, min=0.0, max=35000.0)

    # results = minimize(swerve_cost, params, method='nelder',
    #                    args=(t_slice, ref_traj_slice, actuator_slice))

    # NOTE: The args must be (full_data,) or (full_data, other variables) otherwise it passes the dict itself rather than a tuple
    results = minimize(swerve_cost_multipoint, params, method='nelder',
                       args=(full_data,))

    # Print the parameter results
    param_star = results.params
    # print(f"a: {param_star['a']}\nb: {param_star['b']}\nc: {param_star['c']}")
    print(f"b: {param_star['b']}\nc: {param_star['c']}")

    # Show results for each time slice
    for i in range(len(INIT_TIMES)):
        t0 = INIT_TIMES[i]
        tf = FINAL_TIMES[i]

        # Get a time slice of the data
        sliced_data = slice_data(full_data, t0, tf)

        # Local variables for sliced data
        t_slice = sliced_data['t']
        x_slice = sliced_data['x']
        actuator_slice = sliced_data['actuator']
        ref_traj_slice = sliced_data['ref_traj']

        # Simulate the identified system on the sliced data
        sliced_x0 = list(ref_traj_slice[0, :])
        sliced_sys_traj = simulate_system(t_slice, actuator_slice, sliced_x0, SWERVE_MODEL, param_star)

        # Display results
        print(f"Slice MSE (x): {mean_squared_error(x_slice, sliced_sys_traj[0,:])}")
        plot_results(t_slice, ref_traj_slice, sliced_sys_traj, actuator_slice)

    # Simulate the identified system on the full dataset
    full_x0 = list(full_data['ref_traj'][0, :])
    full_sys_traj = simulate_system(full_data['t'], full_data['actuator'], full_x0, SWERVE_MODEL, param_star)

    # Report MSE and plot full data
    print(f"Full MSE (x): {mean_squared_error(full_data['x'], full_sys_traj[0,:])}")
    plot_results(full_data['t'], full_data['ref_traj'], full_sys_traj, full_data['actuator'])

    
    # Pause after plotting all results
    plt.show(block=False)
    plt.pause(0.001)  # Pause
    input("hit[enter] to end.")
    plt.close('all')


def swerve_cost(params, t, ref_traj, ctrl):
    x0 = list(ref_traj[0, :])
    x_traj = simulate_system(t, ctrl, x0, SWERVE_MODEL, params)

    # Trajectory components
    x_rate = x_traj[1, :]

    # Reference trajectory
    ref_x_rate = ref_traj[:, 1]

    return x_rate - ref_x_rate

def swerve_cost_multipoint(params, data: dict):

    cost = np.array([])

    for i in range(len(INIT_TIMES)):
        t0 = INIT_TIMES[i]
        tf = FINAL_TIMES[i]

        # Slice the data
        s_data = slice_data(data, t0, tf)

        # Get required inputs
        t = s_data['t']
        ctrl = s_data['actuator']
        ref_traj = s_data['ref_traj']

        # Find initial condition and simulate
        x0 = list(ref_traj[0, :])
        x_traj = simulate_system(t, ctrl, x0, SWERVE_MODEL, params)

        # Trajectory components
        x_rate = x_traj[1, :]

        # Reference trajectory
        ref_x_rate = ref_traj[:, 1]

        # Compute cost at each timestep (difference between true x rate and simulated x rate)
        cost_i = x_rate - ref_x_rate
        cost = np.concatenate((cost, cost_i))

    return cost


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