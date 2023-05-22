import numpy as np
from time import time
from scipy.signal import butter, lfilter

def smoothing_factor(t_e, cutoff):
    r = 2 * np.pi * cutoff * t_e
    return r / (r + 1)

def exponential_smoothing(a, x, x_prev):
    return a * x + (1 - a) * x_prev

class OneEuroFilter:
    # https://jaantollander.com/post/noise-filtering-using-one-euro-filter/
    def __init__(self, x0, t0=None, dx0=0.0, min_cutoff=1.0, beta=0.0,
                 d_cutoff=1.0):
        """Initialize the one euro filter."""
        # The parameters.
        self.data_shape = x0.shape
        self.min_cutoff = np.full(x0.shape, min_cutoff)
        self.beta = np.full(x0.shape, beta)
        self.d_cutoff = np.full(x0.shape, d_cutoff)
        # Previous values.
        self.x_prev = x0.astype(np.float64)
        self.dx_prev = np.full(x0.shape, dx0)
        if t0 is None:
            self.t_prev = time()
        else:
            self.t_prev = t0

    def __call__(self, x, t=None):
        """Compute the filtered signal."""
        assert x.shape == self.data_shape
        if t is None:
            t = time()

        t_e = t - self.t_prev
        t_e = np.full(x.shape, t_e)

        # The filtered derivative of the signal.
        a_d = smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = exponential_smoothing(a_d, dx, self.dx_prev)

        # The filtered signal.
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = smoothing_factor(t_e, cutoff)
        x_hat = exponential_smoothing(a, x, self.x_prev)

        # Memorize the previous values.
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat

class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.prev_values = []
        
    def __call__(self, new_value):
        self.prev_values.append(new_value)
        if len(self.prev_values) > self.window_size:
            self.prev_values.pop(0)
        return np.mean(self.prev_values, axis=0)

class MovingMedianFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.prev_values = []
        
    def __call__(self, new_value):
        self.prev_values.append(new_value)
        if len(self.prev_values) > self.window_size:
            self.prev_values.pop(0)
        return np.median(self.prev_values, axis=0)

# class LowPassFilter:
#     def __init__(self, fs=10, low_cut=0.05, order=2):
#         self.fs = fs
#         self.nyq = 0.5 * fs
#         self.low = low_cut / self.nyq
#         self.order = order
#         self.b, self.a = butter(self.order, self.low, 'lowpass', analog=False)

#     def __call__(self, signal):
#         f_state = np.zeros((self.order, signal.shape[1]))
#         y, f_state = lfilter(self.b, self.a, signal, axis=0, zi=f_state)
#         return y
