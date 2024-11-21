import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import matplotlib.animation as animation

# Constants
g = 9.81  # Gravitational acceleration (m/s^2)
k = 10.0  # Spring constant (N/m)
m = 1.0   # Mass (kg)
damping_coefficient = 0.5  # Damping coefficient (kg/s)

# Hexagonal grid parameters
num_masses = 6  # Number of masses in the hexagonal arrangement (simplified for clarity)
positions = np.array([[np.cos(2*np.pi*i/6), np.sin(2*np.pi*i/6)] for i in range(num_masses)])  # Simple hexagon

# Initial conditions
initial_positions = positions  # Starting at hexagonal positions
initial_velocities = np.zeros_like(initial_positions)  # Initially at rest

# Initial conditions for the ODE solver (position and velocity)
initial_conditions = np.concatenate((initial_positions.flatten(), initial_velocities.flatten()))

# Function to compute forces (gravity, spring, and damping)
def spring_force(pos1, pos2):
    # Calculate spring force based on Hooke's law
    displacement = pos2 - pos1
    distance = np.linalg.norm(displacement)
    force_magnitude = k * (distance - 1)  # Assuming rest length of 1m for simplicity
    force_direction = displacement / distance  # Normalize vector
    return force_magnitude * force_direction

def system_of_equations(t, y):
    # Extract positions and velocities from the state vector
    positions = y[:2*num_masses].reshape((num_masses, 2))
    velocities = y[2*num_masses:].reshape((num_masses, 2))
    
    # Compute the net forces
    forces = np.zeros_like(positions)
    
    # Apply gravity force (downward)
    forces[:, 1] -= m * g  # Gravity acts in the negative y direction
    
    # Apply spring forces (pairwise between adjacent masses)
    for i in range(num_masses):
        for j in range(i + 1, num_masses):
            spring_f = spring_force(positions[i], positions[j])
            forces[i] += spring_f
            forces[j] -= spring_f
    
    # Apply damping forces (opposes the velocity)
    damping_forces = -damping_coefficient * velocities
    forces += damping_forces
    
    # Newton's second law: F = ma, so a = F/m
    accelerations = forces / m
    
    # Return the derivatives: velocity and acceleration
    return np.concatenate((velocities.flatten(), accelerations.flatten()))

# Time span for the simulation
t_span = (0, 10)  # 10 seconds
t_eval = np.linspace(0, 10, 500)

# Solve the system of equations using scipy's solve_ivp
sol = solve_ivp(system_of_equations, t_span, initial_conditions, t_eval=t_eval)

# Extract the results
positions_sol = sol.y[:2*num_masses, :].reshape((num_masses, 2, -1))

# Create the figure and axis for the animation
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.set_title('Hexagonal Spring-Mass System Animation')

# Create a scatter plot for the masses
scat = ax.scatter(positions_sol[0, 0, 0], positions_sol[0, 1, 0], color='b', s=100)

# Create a list to store the spring lines
springs = []
for i in range(num_masses):
    for j in range(i + 1, num_masses):
        # Draw a line for each spring between masses i and j
        line, = ax.plot([], [], 'k-', lw=1)  # line object for each spring
        springs.append((i, j, line))

# Function to update the plot for each frame in the animation
def update(frame):
    # Update the positions of the masses
    scat.set_offsets(positions_sol[:, :, frame].T)
    
    # Update the spring lines
    for i, j, line in springs:
        line.set_data([positions_sol[i, 0, frame], positions_sol[j, 0, frame]],
                      [positions_sol[i, 1, frame], positions_sol[j, 1, frame]])
    
    return scat, *[line for _, _, line in springs]

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=len(t_eval), interval=50, blit=True)

# Show the animation
plt.show()
