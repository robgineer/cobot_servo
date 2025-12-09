"""
Signal filters.
"""

import time
import numpy as np
from enum import IntEnum
from scipy.linalg import inv


class KalmanFilterState(IntEnum):
    """Easing up access."""

    VX = 0
    VX_LATENT = 1
    VY = 2
    VY_LATETNT = 3


class KalmanFilter(object):
    """
    Std. Kalman Filter implementation. Works with various number of states.
    Implementation details:
    https://github.com/robgineer/probabilistic_robotics/blob/master/02_kalman_filter.ipynb
    """

    def __init__(self, P, F, R, Q, H, init_state=np.zeros((2)), num_states=2):
        self.P = P  # state estimate
        self.F = F  # state transition
        self.R = R  # measurement noise
        self.Q = Q  # process cov.
        self.H = H  # measurement function

        self.num_states = num_states
        self.x = init_state
        self.last_time = time.time()

    def predict(self, current_time=None):
        """
        Predict step of Kalman Filter.
        Args:
           current_time: time (python time type) transition matrix update
        """

        if current_time == None:
            current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        # update state transition dt for n==2 and n==4
        if self.num_states == 4:
            self.F = np.array(
                [[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]]
            )
        elif self.num_states == 2:
            self.F = np.array([[1, dt], [0, 1]])
        # predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """Update step of Kalman Filter."""
        # error
        y = z - self.H @ self.x
        # innovation
        S = self.H @ self.P @ self.H.T + self.R
        # Kalman gain
        K = self.P @ self.H.T @ inv(S)
        # update state and covariance
        self.x = self.x + (K @ y)
        self.P = (np.eye(self.num_states, self.num_states) - K @ self.H) @ self.P


class AlphaFilter(object):
    """Simple alpha filter."""

    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.previous_alpha_filtered_input_signal = None

    def filter(self, raw_signal):
        if self.previous_alpha_filtered_input_signal == None:
            self.previous_alpha_filtered_input_signal = raw_signal
        filtered_signal = (
            raw_signal * self.alpha
            + (1 - self.alpha) * self.previous_alpha_filtered_input_signal
        )
        self.previous_alpha_filtered_input_signal = filtered_signal

        return filtered_signal


class SmoothVelocityCalculator(object):
    """
    This is a simple one dimensional velocity calculator.
    Implemented as a class in order to store the state.
    Applies an alpha filter.
    """

    def __init__(self, alpha=0.5):
        self.previous_time = None
        self.previous_alpha_filtered_input_signal = None
        self.alpha_filter = AlphaFilter(alpha)

    def reset(self):
        """Reset internal state."""
        self.__init__()

    def calculate_velocity_from_signal(self, input_signal, current_time):
        """
        Calculates the velocity.

        Returns velocity if previous time and signal are available.
        0 otherwise.
        """

        alpha_filtered_input_signal = self.alpha_filter.filter(input_signal)

        if (
            self.previous_time == None
            or self.previous_alpha_filtered_input_signal == None
        ):
            # last measurement not available
            self.previous_alpha_filtered_input_signal = alpha_filtered_input_signal
            self.previous_time = current_time
            return 0.0

        delta = alpha_filtered_input_signal - self.previous_alpha_filtered_input_signal
        dt = current_time - self.previous_time
        velocity = 0.0
        if dt > 0.0:
            velocity = delta / dt

        self.previous_alpha_filtered_input_signal = alpha_filtered_input_signal
        self.previous_time = current_time

        return velocity
