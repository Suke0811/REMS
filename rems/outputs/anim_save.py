# importing required libraries
from matplotlib import pyplot as plt
import numpy as np
import matplotlib.animation as animation
from IPython import display

# initializing a figure
fig = plt.figure()

# labeling the x-axis and y-axis
axis = plt.axes(xlim=(0, 1000), ylim=(0, 1000))

# lists storing x and y values
x, y = [], []

line, = axis.plot(0, 0)


def animate(frame_number):
    x.append(frame_number)
    y.append(frame_number)
    line.set_xdata(x)
    line.set_ydata(y)
    return line,


anim = animation.FuncAnimation(fig, animate, frames=1001,
                               interval=50, blit=True)
fig.suptitle('Straight Line plot', fontsize=14)

# saving to m4 using ffmpeg writer
writervideo = animation.PillowWriter(fps=30)
anim.save('increasingStraightLine.gif', writer=writervideo)


plt.close()
