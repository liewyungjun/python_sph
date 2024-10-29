import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

def onclick(event):
    if event.button == 1:  # Left click
        if event.inaxes:
            points.append([event.xdata, event.ydata])
            ax.plot(event.xdata, event.ydata, 'ro')  # Plot point
            plt.draw()
            
            if len(points) == 2:  # After second click, create rectangle
                x1, y1 = points[0]
                x2, y2 = points[1]
                width = x2 - x1
                height = y2 - y1
                rect = Rectangle((x1, y1), width, height, alpha=0.5)
                ax.add_patch(rect)
                # Create all four corners
                corner = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                # Save corners
                corners.append(corner)
                # with open('polygon_points.txt', 'a') as f:
                #     f.write(str(corner) + '\n')
                points.clear()  # Clear points for next rectangle
                plt.draw()

def onkey(event):
    global points
    if event.key == 'home':  # Reset plot when home key is pressed
        ax.clear()
        ax.set_xlim(x_limits)
        ax.set_ylim(y_limits)
        points = []
        global corners
        corners = []
        plt.draw()
    elif event.key == 'a':  # Save points when end is pressed
        with open(f'maps/{mapname}.txt', 'a') as f:
            for i in corners:
                f.write(str(i) + '\n')

def on_close(event):
    if corners:
        with open(f'maps/{mapname}.txt', 'a') as f:
            f.write('[')
            for i in range(len(corners)-1):
                f.write(str(corners[i]) + ',\n')
            f.write(str(corners[-1]) + ']\n')

#plotsize setup
ratio = (3,4)
plotSize = 50.0
axesScaling = 10 #size of axes scaling factor e.g. 10 units/axesScaling = plot cm size
mapname = "maze"

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
corners = []
fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', onkey)
fig.canvas.mpl_connect('close_event', on_close)
plt.show()