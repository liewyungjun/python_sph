from matplotlib.collections import LineCollection
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create figure and axis
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)

# Sample data points (replace with your x and y lists)
x = [0, 2, -1, 3]  # example x coordinates
y = [1, -2, 3, 0]  # example y coordinates

def animate(frame):
    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    current_x = [xi + frame for xi in x]
    lines = np.array([[(0, 0), (x, y)] for x, y in zip(current_x, y)])
    lc = LineCollection(lines, colors='b')
    ax.add_collection(lc)
anim = FuncAnimation(fig, animate, frames=10, interval=500, repeat=True)
plt.show()