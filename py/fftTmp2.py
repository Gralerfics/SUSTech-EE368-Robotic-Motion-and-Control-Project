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

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.15, bottom=0.35)

canvas = ax.figure.canvas
line, = ax.plot([], [], 'b')

def fourier_series(x, y, n):
    # 计算傅里叶级数的系数
    coefficients = np.fft.fft(y)
    
    # 保留指定数量的谐波
    coefficients[n+1:-n] = 0
    print(coefficients)
    
    # 通过傅里叶逆变换得到拟合曲线
    fitted_curve = np.fft.ifft(coefficients).real
    
    return x, fitted_curve

def update_plot(n):
    # 获取拖动条的当前值
    n_harmonics = int(n)
    
    # 计算傅里叶级数拟合曲线
    fitted_x, fitted_y = fourier_series(x, y, n_harmonics)
    
    # 更新绘图数据
    line.set_data(fitted_x, fitted_y)
    ax.relim()
    ax.autoscale_view()
    canvas.draw_idle()

def slider_callback(val):
    update_plot(val)

def reset_button_callback(event):
    slider.set_val(0)

# 创建拖动条
ax_slider = plt.axes([0.15, 0.2, 0.7, 0.03])
slider = Slider(ax_slider, '谐波数', 0, len(x)//2, valinit=0, valstep=1)
slider.on_changed(slider_callback)

# 创建重置按钮
ax_reset = plt.axes([0.8, 0.025, 0.1, 0.04])
reset_button = Button(ax_reset, '重置', hovercolor='0.975')
reset_button.on_clicked(reset_button_callback)

plt.show()
