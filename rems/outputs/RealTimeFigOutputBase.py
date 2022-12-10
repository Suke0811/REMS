import time

from matplotlib import pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from rems.outputs import OutputBase
from rems.outputs.PlotlyHelper import PlotlyHelper
import numpy as np

FORMAT_XY = dict(x=float, y=float)
FORMAT_XYTh = dict(x=float, y=float, th_z=float)
DEFAULT_AXIS_SIZE = 0.1
SCALE = 0.04
MARGIN = 0.1

class RealtimeFig(OutputBase):
    def __init__(self, tail_length=float('inf'), fps_limit=15):
        super().__init__()
        self.tail_length = tail_length
        self.fps_limit = fps_limit
        self.last_update = 0.0
        self.fig = None


    def process(self, state, inpt, outpt, timestamp, info):
        super().process(state, inpt, outpt, timestamp, info)
        if self.fig is None:
            self._create_canvas()
        if self._under_fps_limit(timestamp):
            self.last_update = timestamp
            plot_data = self.get_plot_list(self._states, FORMAT_XYTh)
            last_state = round(self._states[-1], 2)
            self._update_canvas(plot_data, f"state: {last_state.__str__()}")

    def make_output(self):
        """make proper output from the data"""
        if len(self._timestamps) <= 2:
            return

        # Currently do nothing but future save png or so

    def _under_fps_limit(self, timestamp):
        return bool((1/(timestamp - self.last_update)) <= self.fps_limit)

    def _create_canvas(self):
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], lw=3)
        self.ann = self.ax.annotate('', xy=(0, 0), arrowprops=dict(arrowstyle="<-", color='red', lw=3))
        self.text = self.ax.text(0.0, 0.0, "")
        self.fig.canvas.draw()
        plt.autoscale(True)
        plt.rcParams["keymap.quit"] = "esc"
        plt.show(block=False)

    def _update_canvas(self, line=None, text=None):
        x_l, y_l = self.ax.get_xlim(), self.ax.get_ylim()
        x_l, y_l = (x_l[1]-x_l[0]), (y_l[1]-y_l[0])
        if line is not None:
            x,y,th = line
            self.line.set_data(*(x,y))
            self.ann.remove()
            x_l, y_l = self._get_axis_size(x,y)
        if text is not None:
            self.text.set_position((line[0][-1] + SCALE * x_l, line[1][-1] + SCALE * y_l))
            self.text.set_text(text)

        self.fig.canvas.draw()
        self.ax.set_xlim([min(line[0])-MARGIN*x_l, max(line[0])+MARGIN*x_l])
        self.ax.set_ylim([min(line[1])-MARGIN*y_l, max(line[1])+MARGIN*y_l])

        self.fig.canvas.flush_events()

    def _get_axis_size(self, x, y):
        l = []
        for v in (x, y):
            l.append(max(v) - min(v))
        return tuple(l)


    def get_data_range(self, data):
        # get data from the latest to tail_lengh
        states = []
        current_t = self._timestamps[-1]['timestamp']
        for timestamp, state in zip(reversed(self._timestamps), reversed(data)):
            if (current_t-self.tail_length) >= timestamp['timestamp']:
                break
            states.append(state)
        return list(reversed(states))

    def get_plot_list(self, data, format):
        data = self.get_data_range(data)
        list_data = list(map(lambda d: d.filter(format).list(), data))
        ret_list = []
        for i in range(len(format)):
            ret_list.append([d[i] for d in list_data])
        return tuple(ret_list)

    def calc_fps(self):
        st = self._timestamps[0]['timestamp']
        et = self._timestamps[-1]['timestamp']
        fps = 1/((et-st)/(len(self._timestamps)-1))
        return fps



