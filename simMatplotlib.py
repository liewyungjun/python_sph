import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_aspect('equal')

# Initialize the balls
ball1, = ax.plot([], [], 'ro', markersize=20)
ball2, = ax.plot([], [], 'bo', markersize=20)

# Physics parameters
g = 9.8  # Acceleration due to gravity
e = 0.8  # Coefficient of restitution
dt = 0.01  # Time step

# Initial conditions
x01, y01 = 0, 0.9  # Initial position for ball 1
vx01, vy01 = 0.5, 0  # Initial velocity for ball 1
x02, y02 = -0.5, 0.5  # Initial position for ball 2
vx02, vy02 = 0.3, 0.2  # Initial velocity for ball 2

# Lists to store position and velocity
x1, y1 = [x01], [y01]
vx1, vy1 = [vx01], [vy01]
x2, y2 = [x02], [y02]
vx2, vy2 = [vx02], [vy02]

def update_ball(x, y, vx, vy):
    x.append(x[-1] + vx[-1] * dt)
    y.append(y[-1] + vy[-1] * dt - 0.5 * g * dt**2)
    vx.append(vx[-1])
    vy.append(vy[-1] - g * dt)
    
    # Check for collision with ground
    if y[-1] < 0:
        y[-1] = 0
        vy[-1] = -e * vy[-1]
    
    # Check for collision with walls
    if abs(x[-1]) > 1:
        x[-1] = np.sign(x[-1])
        vx[-1] = -e * vx[-1]

def update(frame):
    # Update position and velocity for both balls
    update_ball(x1, y1, vx1, vy1)
    update_ball(x2, y2, vx2, vy2)
    
    # Update ball positions
    ball1.set_data(x1[-1], y1[-1])
    ball2.set_data(x2[-1], y2[-1])
    return ball1, ball2,

# Create the animation
anim = animation.FuncAnimation(fig, update, frames=500, interval=20, blit=True)

plt.title("Two Bouncing Balls Simulation")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
