import plotly.graph_objects as go

import numpy as np

DEFAULT_MARGIN = 0.1
FPS_LIMIT = 10
SCALE = 100

class PlotlyHelper:
    def __init__(self, save_html_path=None):
        self.save_html_path = save_html_path

    def anim_path_dot(self, x, y, th=None, fps=1, title="Animation"):

        if th is None:
            th = np.zeros(len(x))

        x_range, y_range = self.axis_limit(x, y, DEFAULT_MARGIN)
        x_l, y_l = self._get_axis_size(x_range, y_range)

        duration = round(1000/fps)
        n = 1   # if need to drop some data points
        N = list(range(len(x))[1::n])
        N.append(len(x)-1)
        fig = go.Figure(
            data=[go.Scatter(x=x, y=y,
                             mode="lines",
                             line=dict(width=2, color="blue")),
                  go.Scatter(x=x, y=y,
                             mode="lines",
                             line=dict(width=2, color="blue"))
                  ],
            layout=go.Layout(
                xaxis=dict(range=x_range, autorange=False, zeroline=False, title='x, [m]'),
                yaxis=dict(range=y_range, autorange=False, zeroline=False, title='y, [m]'),
                title_text=title, hovermode="closest",
                updatemenus=[dict(type="buttons",
                                  buttons=[dict(label="Play", method="animate",
                                                args=[None, {"frame": {"duration": duration, "redraw": False},
                                                             "mode": "immediate",
                                                             "fromcurrent": True, "transition": {"duration": duration,
                                                                                                 "easing": "quadratic-in-out"}}]),
                                           dict(label='Pause', method="animate",
                                                args=[[None], {"frame": {"duration": 0, "redraw": False},
                                                               "mode": "immediate",
                                                               "transition": {"duration": 0}}])
                                           ])]),
            frames=[go.Frame(
                data=[go.Scatter(
                    x=[x[k]],
                    y=[y[k]],
                    mode="text",
                    text=f"State: x: {round(x[k],2)}, y: {round(y[k], 2)}, th: {round(th[k], 2)}",
                    textposition="top right",
                    #marker=dict(color="red", size=7)
                     )],
                layout=go.Layout(
                    annotations=[
                        dict(
                            x=x[k],
                            y=y[k],
                            # axref="x",
                            # ayref="y",
                           # ax=x[k] + 0.2 * np.cos(th[k]),
                           # ay=y[k] + 0.2 * np.sin(th[k]),
                            ax = SCALE * x_l * np.cos(th[k]),
                            ay = - SCALE * y_l * np.sin(th[k]),
                            #text=f"State: x: {round(x[k], 2)}, y: {round(y[k], 2)}, th: {round(th[k], 2)}",
                            arrowside='end+start',
                            showarrow=True,
                            arrowsize=2,
                            arrowcolor='red',
                            arrowhead=6,
                            startarrowsize=2,
                            startarrowhead=4,


                        )], )
            ) for k in N],)
        fig.update_yaxes(scaleanchor="x", scaleratio=1)
        if self.save_html_path is not None:
            fig.write_html(self.save_html_path)
        fig.show()

    def _get_axis_size(self, x, y):
        l = []
        for v in (x, y):
            l.append(max(v) - min(v))
        return tuple(l)


    def axis_limit(self, x, y, margin):
        def min_max(v):
            vmin = min(v)
            vmax = max(v)
            vdist = vmax - vmin
            if vdist == 0.0:
                vdist = 1.0
                vmax = 1.0
            return [vmin - margin * vdist, vmax + margin * vdist]
        x_limit = min_max(x)
        y_limit = min_max(y)
        return x_limit, y_limit

    def __test_data(self, N=1000):
        # Generate curve data
        t = np.linspace(-1, 1, N)
        x = t + t ** 2
        y = t - t ** 2
        xm = np.min(x) - 1.5
        xM = np.max(x) + 1.5
        ym = np.min(y) - 1.5
        yM = np.max(y) + 1.5
        return x, y

