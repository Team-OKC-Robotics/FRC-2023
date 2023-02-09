
import numpy as np

def mean_squared_error(ref_x, x):
    # Compute the element-wise squared error
    squared_err = np.square(ref_x - x)

    # Take the mean
    mse = np.mean(squared_err)

    return mse