import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.interpolate import interp1d
import time

x, y = None, None

class PointRecorder:
    def __init__(self):
        self.points = []
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 10)  # 根据需要调整 x 轴范围
        self.ax.set_ylim(0, 10)  # 根据需要调整 y 轴范围
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.current_points = []
        self.motioning = False
        plt.show()

    def on_press(self, event):
        if event.button == 1:  # 检查是否为鼠标左键按下
            self.motioning = True

    def on_release(self, event):
        if event.button == 1:  # 检查是否为鼠标左键释放
            self.motioning = False

    def on_motion(self, event):
        if self.motioning:
            x = event.xdata
            y = event.ydata
            if x is not None and y is not None:
                self.points.append((x, y))
                self.plot_points([(x, y)])

    def on_key_press(self, event):
        if event.key == 'q':  # 检查是否按下 'q' 键
            self.fig.canvas.mpl_disconnect(self.cid_press)
            self.fig.canvas.mpl_disconnect(self.cid_release)
            self.fig.canvas.mpl_disconnect(self.cid_motion)
            self.fig.canvas.mpl_disconnect(self.cid_key)
            plt.close()
            self.save_points()

    def plot_points(self, points):
        xs, ys = zip(*points)
        self.ax.plot(xs, ys, 'ro')
        self.fig.canvas.draw()

    def save_points(self):
        global x, y
        x, y = zip(*self.points)
        x = np.array(list(x))
        y = np.array(list(y))

point_recorder = PointRecorder()


