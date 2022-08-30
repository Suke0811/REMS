import time

from matplotlib import pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from sim.outputs import OutputBase
from sim.outputs.PlotlyHelper import PlotlyHelper
import numpy as np

FORMAT_XY = dict(x=float, y=float)
FORMAT_XYTh = dict(x=float, y=float, th_z=float)
DEFAULT_AXIS_SIZE = 0.1
SCALE = 0.04
MARGIN = 0.1

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

            plot_data = self.get_plot_list(self._states, FORMAT_XYTh)
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
        #marker = (3, 0, i * 90), markersize = 20, linestyle = 'None')
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], lw=3)
        self.ann = self.ax.annotate('', xy=(0,0),
                                    arrowprops=dict(arrowstyle="<-", color='red', lw=3))
        self.text = self.ax.text(0.0, 0.0, "")
        self.fig.canvas.draw()
        self.ax_cache = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        plt.autoscale(True)
        plt.show(block=False)

    def _update_canvas(self, line=None, text=None):
        x_l, y_l = self.ax.get_xlim(), self.ax.get_ylim()
        x_l, y_l = (x_l[1]-x_l[0]), (y_l[1]-y_l[0])
        if line is not None:
            x,y,th = line
            self.line.set_data(*(x,y))
            self.ann.remove()
            x_l, y_l = self._get_axis_size(x,y)
            self.ann = self.ax.annotate('', xy=(x[-1], y[-1]),
                        xytext=(x[-1] + SCALE*x_l*np.cos(th[-1]), y[-1] + SCALE*y_l*np.sin(th[-1])),
                        arrowprops=dict(arrowstyle="<-",
                         color='red',
                         lw=3,
                         ls='--'))
        if text is not None:
            
            self.text.set_position((line[0][-1] + SCALE * x_l, line[1][-1] + SCALE * y_l))
            self.text.set_text(text)

        if self.use_cache:
            # restore background
            self.fig.canvas.restore_region(self.ax_cache)

            # redraw just the points
            self.ax.draw_artist(self.line)
            self.ax.draw_artist(self.text)

            # fill in the axes rectangle
            self.fig.canvas.blit(self.ax.bbox)

        else:
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

    def generate_video(self):
        # initializing a figure
        fig = plt.figure()
        axis = plt.axes()
        ann = axis.annotate('', xy=(0,0), arrowprops=dict(arrowstyle="<-",
                         color='red',
                         lw=3,
                         ls='--'))
        ann_list =[ann]
        # labeling the x-axis and y-axis
        #axis = plt.axes(xlim=(0, 1000), ylim=(0, 1000))

        # lists storing x and y values
        x, y, th = [], [], []

        states_x, states_y, state_th = self.get_plot_list(self._states, FORMAT_XYTh)

        axis.set_xlim([min(states_x), max(states_x)])
        axis.set_ylim([min(states_y), max(states_y)])
        x_l, y_l = self._get_axis_size(states_x, states_y)
        line, = axis.plot(0, 0)
        text = axis.text(0.0,0.0,'')

        def animate(frame_number):
            x.append(states_x[frame_number])
            y.append(states_y[frame_number])
            th.append(state_th[frame_number])
            line.set_xdata(x)
            line.set_ydata(y)
            self.text.set_position((states_x[frame_number] + SCALE *x_l, states_y[frame_number]+ SCALE * y_l))
            last_state = round(self._states[frame_number], 2)
            self.text.set_text(f"state: {last_state.__str__()}")
            ann_list.pop().remove()
            ann = axis.annotate('', xy=(states_x[frame_number], states_y[frame_number]),
                                    xytext=(states_x[frame_number]+SCALE * x_l * np.cos(state_th[frame_number]),
                                            states_y[frame_number]+SCALE * y_l * np.sin(state_th[frame_number])),
                                    arrowprops=dict(arrowstyle="<-",
                         color='red',
                         lw=3,
                         ls='--'))
            ann_list.append(ann)
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
