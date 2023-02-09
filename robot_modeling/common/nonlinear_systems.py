
import numpy as np

def second_order_sys_x2_deadband(t, x, u, params):
    # Apply deadband "d"
    if abs(u) < params['d']:
        u = 0

    # Parameters for x
    b = params['b']
    c = params['c']

    # Linear state equation
    A = np.array([[0, 1], [0, b]])
    B = np.array([[0], [c]])

    # Make x into a column vector for matrix multiplication
    X = x[None].T

    # System update
    x_dot = (A@X + B*u).T.flatten()

    return x_dot