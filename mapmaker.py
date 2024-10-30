
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

def onclick(event):
    if event.button == 1:  # Left click
        if event.inaxes:
            points.append([event.xdata, event.ydata])
            ax.plot(event.xdata, event.ydata, 'ro')  # Plot point
            plt.draw()
    elif event.button == 3:  # Right click
        if len(points) >= 3:  # Need at least 3 points for a polygon
            poly = Polygon(np.array(points), alpha=0.5)
            ax.add_patch(poly)
            temp = [[round(x[0], 2), round(x[1], 2)] for x in points]
            polys.append(temp)
            # # Save points before clearing
            # with open('maps/polygon_points.txt', 'a') as f:
            #     f.write(str(points) + '\n')
            points.clear()  # Clear points for next polygon
            plt.draw()
def onkey(event):
    global points
    if event.key == 'home':  # Reset plot when home key is pressed
        ax.clear()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        points = []
        global polys
        polys = []
        plt.draw()
    elif event.key == 'a':  # Save points when end is pressed
        with open(f'maps/{mapname}.txt', 'a') as f:
            for i in polys:
                f.write(str(i) + '\n')
def on_close(event):
    if polys:
        with open(f'maps/{mapname}.txt', 'a') as f:
            f.write('[')
            for i in range(len(polys)-1):
                f.write(str(polys[i]) + ',\n')
            f.write(str(polys[-1]) + ']\n')
# Create figure and connect click event
#plotsize setup
ratio = (3,4)
plotSize = 50.0
axesScaling = 10 #size of axes scaling factor e.g. 10 units/axesScaling = plot cm size

#Simulation graphical size calculations
x_limits = (0, ratio[0]*plotSize)
y_limits = (0, ratio[1]*plotSize)
figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
points_per_unit = inches_per_unit * 72  # convert to points

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
ax.set_xlim(x_limits)
ax.set_ylim(y_limits)
ax.set_aspect('equal')
plt.xticks(np.arange(x_limits[0], x_limits[1], 5.0))
plt.title(f"SPH Simulation\n")
ax.set_position([0.1, 0.1, 0.8, 0.8])

points = []
polys = []
mapname = "poly"
fig.canvas.mpl_connect('button_press_event', onclick)
# Connect keyboard event

fig.canvas.mpl_connect('close_event', on_close)
fig.canvas.mpl_connect('key_press_event', onkey)
plt.show()
