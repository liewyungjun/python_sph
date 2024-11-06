
from vpython import *

# Set up the scene
scene = canvas(title='Bouncing Ball with Gravity', width=800, height=600, center=vector(0,0,0), background=color.white)

# Create the ball
ball = sphere(pos=vector(0, 5, 0), radius=0.5, color=color.red)

# Set initial velocity and acceleration
ball.velocity = vector(0, 0, 0)
gravity = vector(0, -9.8, 0)

# Create the floor
floor = box(pos=vector(0, -5, 0), size=vector(10, 0.1, 10), color=color.green)

# Set up the simulation
dt = 0.01
t = 0

# Main animation loop
while True:
    rate(100)  # Limit the animation speed
    
    # Update ball position and velocity
    ball.pos += ball.velocity * dt
    ball.velocity += gravity * dt
    
    # Check for collision with the floor
    if ball.pos.y - ball.radius <= floor.pos.y + floor.size.y/2:
        ball.velocity.y = -ball.velocity.y * 0.9  # Bounce with some energy loss
        ball.pos.y = floor.pos.y + floor.size.y/2 + ball.radius  # Prevent sinking
    
    t += dt
