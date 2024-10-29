import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation

def create_cuboid(center, size):
    x, y, z = center
    dx, dy, dz = size
    vertices = np.array([
        [x-dx/2, y-dy/2, z-dz/2],
        [x+dx/2, y-dy/2, z-dz/2],
        [x+dx/2, y+dy/2, z-dz/2],
        [x-dx/2, y+dy/2, z-dz/2],
        [x-dx/2, y-dy/2, z+dz/2],
        [x+dx/2, y-dy/2, z+dz/2],
        [x+dx/2, y+dy/2, z+dz/2],
        [x-dx/2, y+dy/2, z+dz/2]
    ])
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[0], vertices[3], vertices[7], vertices[4]]
    ]
    return faces

# Create figure and 3D axes
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Create multiple cuboids with different positions, sizes and colors
cuboids = [
    ((0, 0, 0), (2, 2, 4), 'red'),
    ((4, 0, 0), (1, 3, 2), 'blue'),
    ((2, 3, 0), (2, 1, 3), 'green'),
    ((-2, -2, 0), (1.5, 1.5, 2), 'yellow')
]

# Plot each cuboid
for center, size, color in cuboids:
    faces = create_cuboid(center, size)
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, alpha=0.6))

# Set axis labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-4, 6)
ax.set_ylim(-4, 6)
ax.set_zlim(0, 6)

# Create moving point
point, = ax.plot([], [], [], 'ko', markersize=10)
# Create trajectory line
trajectory, = ax.plot([], [], [], 'b-', alpha=0.5)

# Update animation function
def animate(frame):
    t = frame / 100  # Scale time for smoother animation
    x = -4 + 10 * t  # Move from x=-4 to x=6
    y = 2 * np.sin(2 * np.pi * t)  # Sinusoidal motion in y
    z = 3 + np.cos(2 * np.pi * t)  # Sinusoidal motion in z
    
    # Generate trajectory points up to current time
    times = np.linspace(0, t, int(frame + 1))
    trajectory_x = -4 + 10 * times
    trajectory_y = 2 * np.sin(2 * np.pi * times)
    trajectory_z = 3 + np.cos(2 * np.pi * times)
    
    # Update point position
    point.set_data([x], [y])
    point.set_3d_properties([z])
    
    # Update trajectory
    trajectory.set_data(trajectory_x, trajectory_y)
    trajectory.set_3d_properties(trajectory_z)
    
    return point, trajectory

# Create animation
ani = animation.FuncAnimation(fig, animate, frames=100, 
                            interval=50, blit=True)

# Add a title
plt.title('3D Cuboids Plot with Moving Point')

# Show the plot
plt.show()