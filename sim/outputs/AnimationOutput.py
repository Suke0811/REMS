import time

from matplotlib import pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from sim.outputs import OutputBase
from sim.outputs.PlotlyHelper import PlotlyHelper

FORMAT_XY = dict(x=float, y=float)
FORMAT_XYTh = dict(x=float, y=float, th=float)

class AnimationOutput(OutputBase):
    def __init__(self, filepath, tail_length=float('inf'), fps_limit=15):
        super().__init__()
        self.filepath = filepath
        self.tail_length = tail_length
        self.fps_limit = fps_limit
        self.last_update = 0.0
        self.use_cache = False
        self.fig = None

    def process(self, state, inpt, outpt, timestamp, info):
        super().process(state, inpt, outpt, timestamp, info)
        if self.fig is None:
            self._create_canvas()
        if self._under_fps_limit(timestamp):
            self.last_update = timestamp

            plot_data = self.get_plot_list(self._states, FORMAT_XY)
            last_state = round(self._states[-1], 2)
            self._update_canvas(plot_data, f"state: {last_state.__str__()}")

    def make_output(self):
        """make proper output from the data"""
        if len(self._timestamps) <= 2:
            return
        p = PlotlyHelper()
        p.anim_path_dot(*self.get_plot_list(self._states, FORMAT_XYTh), fps=self.calc_fps())
        self.generate_video()

    def _under_fps_limit(self, timestamp):
        return bool((1/(timestamp - self.last_update)) <= self.fps_limit)

    def _create_canvas(self):
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], lw=3)
        self.text = self.ax.text(0.0, 0.0, "")
        self.fig.canvas.draw()
        self.ax_cache = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        plt.autoscale(True)
        plt.show(block=False)

    def _update_canvas(self, line=None, text=None):
        if line is not None:
            self.line.set_data(*line)
        if text is not None:
            self.text.set_position((line[0][0], line[1][0]))
            self.text.set_text(text)

        if self.use_cache:
            # restore background
            self.fig.canvas.restore_region(self.ax_cache)

            # redraw just the points
            self.ax.draw_artist(self.line)
            self.ax.draw_artist(self.text)

            # fill in the axes rectangle
            self.fig.canvas.blit(self.ax.bbox)

            self.ax.set_xlim([min(line[0]), max(line[0])])
            self.ax.set_ylim([min(line[1]), max(line[1])])
        else:
            self.fig.canvas.draw()
            self.ax.set_xlim([min(line[0]), max(line[0])])
            self.ax.set_ylim([min(line[1]), max(line[1])])

        self.fig.canvas.flush_events()


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
        x = [d[0] for d in list_data]
        y = [d[1] for d in list_data]
        return (x, y)

    def generate_video(self):
        # initializing a figure
        fig = plt.figure()
        axis = plt.axes()

        # labeling the x-axis and y-axis
        #axis = plt.axes(xlim=(0, 1000), ylim=(0, 1000))

        # lists storing x and y values
        x, y = [], []

        states_x, states_y = self.get_plot_list(self._states, FORMAT_XY)
        axis.set_xlim([min(states_x), max(states_x)])
        axis.set_ylim([min(states_y), max(states_y)])
        line, = axis.plot(0, 0)

        def animate(frame_number):
            x.append(states_x[frame_number])
            y.append(states_y[frame_number])
            line.set_xdata(x)
            line.set_ydata(y)
            return line,
        fps = self.calc_fps()

        anim = animation.FuncAnimation(fig, animate, frames=len(self._timestamps),
                                       interval=round(1000*1/fps), blit=True)
        fig.suptitle(self.filepath, fontsize=14)

        # saving to m4 using ffmpeg writer
        writervideo = animation.PillowWriter(fps=round(fps))
        p = Path(self.filepath).parent.mkdir(parents=True, exist_ok=True)  # create dir if it doesn't exist
        anim.save(self.filepath, writer=writervideo)
        plt.close()

    def calc_fps(self):
        st = self._timestamps[0]['timestamp']
        et = self._timestamps[-1]['timestamp']
        fps = 1/((et-st)/(len(self._timestamps)-1))
        return fps



if __name__ == '__main__':
    a = AnimationOutput('test.gif')
    a._create_canvas()
    import numpy as np
    x =[]
    y=[]
    ls = np.linspace(0, 2, 100)
    for i in np.arange(100):

        x.append(ls[i])
        y.append(np.sin(2 * np.pi * (ls[i] - 0.01 * i)))
        a._update_canvas((x,y), 'hi')
        time.sleep(0.001)
    a.generate_video()
