import time
from collections import deque
import matplotlib.pyplot as plt


class LivePlotter(object):
    """Helper class to visualize live values of signals.
    Plots created for:
        1. x,y values
        2. kalman filtered vx, vy values
    """

    def __init__(self):
        self.MAX_POINTS = 100  # history size
        # define buffers
        self.time_buffer = deque(maxlen=self.MAX_POINTS)
        self.x_buffer = deque(maxlen=self.MAX_POINTS)
        self.y_buffer = deque(maxlen=self.MAX_POINTS)
        self.kf_vx_buffer = deque(maxlen=self.MAX_POINTS)
        self.kf_vy_buffer = deque(maxlen=self.MAX_POINTS)

        # define plot lines
        # we will display the values of x,y and vx,vy
        # along the y axis
        # their historical values will be plottet
        # along the x axis
        plt.ion()  # interactive mode
        self.fig, self.ax = plt.subplots()
        (self.line_x,) = self.ax.plot([], [], "m-", label="X")
        (self.line_y,) = self.ax.plot([], [], "b-", label="Y")
        (self.line_kf_vx,) = self.ax.plot([], [], "g-", label="KF VX")
        (self.line_kf_vy,) = self.ax.plot([], [], "r-", label="KF VY")

        self.ax.set_xlim(0, self.MAX_POINTS)
        # x represented with range [0,1]
        # y is represented with range [-1,0]
        # => limit y axis to these values
        self.ax.set_ylim(-1, 1)
        self.ax.legend()

        self.last_plot_time = time.time()

    def update_plot(self, current_time, x, y, kf_vx, kf_vy):
        """Updates and plots the buffers"""
        # update buffer
        self.time_buffer.append(current_time)
        self.x_buffer.append(x)
        self.y_buffer.append(y)
        self.kf_vx_buffer.append(kf_vx)
        self.kf_vy_buffer.append(kf_vy)
        # update plot lines
        if current_time - self.last_plot_time > 0.05:
            self.line_x.set_ydata(self.x_buffer)
            self.line_x.set_xdata(range(len(self.x_buffer)))
            self.line_y.set_ydata(self.y_buffer)
            self.line_y.set_xdata(range(len(self.y_buffer)))
            self.line_kf_vx.set_ydata(self.kf_vx_buffer)
            self.line_kf_vx.set_xdata(range(len(self.kf_vx_buffer)))
            self.line_kf_vy.set_ydata(self.kf_vy_buffer)
            self.line_kf_vy.set_xdata(range(len(self.kf_vy_buffer)))
            # update scale based on min / max values
            self.ax.relim()
            self.ax.autoscale_view()
            # plot
            plt.draw()
            self.last_plot_time = time.time()
            plt.pause(0.0001)
