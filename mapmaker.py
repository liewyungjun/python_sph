
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
            # Save points before clearing
            with open('maps/polygon_points.txt', 'a') as f:
                f.write(str(points) + '\n')
            points.clear()  # Clear points for next polygon
            plt.draw()
def onkey(event):
    global points
    if event.key == 'home':  # Reset plot when home key is pressed
        ax.clear()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        points = []
        plt.draw()
    elif event.key == 'a':  # Save points when end is pressed
        if points:
            with open('maps/polygon_points.txt', 'a') as f:
                f.write(str(points) + '\n')

# Create figure and connect click event
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
points = []
fig.canvas.mpl_connect('button_press_event', onclick)
# Connect keyboard event
fig.canvas.mpl_connect('key_press_event', onkey)
plt.show()
