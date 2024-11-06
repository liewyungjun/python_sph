
import pymunk
import pymunk.pygame_util
import pygame
import sys

# Initialize Pygame and create a window
pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Bouncing Ball Simulation")

# Create a Pymunk space
space = pymunk.Space()
space.gravity = (0, 900)  # Set gravity (y-axis points downward)

# Create a ball
mass = 1
radius = 15
moment = pymunk.moment_for_circle(mass, 0, radius)
ball_body = pymunk.Body(mass, moment)
ball_body.position = 300, 100
ball_shape = pymunk.Circle(ball_body, radius)
ball_shape.elasticity = 0.9
ball_shape.friction = 0.4
space.add(ball_body, ball_shape)

# Create static lines for the ground and walls
static_lines = [
    pymunk.Segment(space.static_body, (50, 550), (550, 550), 5),  # Ground
    pymunk.Segment(space.static_body, (50, 50), (50, 550), 5),    # Left wall
    pymunk.Segment(space.static_body, (550, 50), (550, 550), 5),  # Right wall
]
for line in static_lines:
    line.elasticity = 0.9
    line.friction = 0.5
space.add(*static_lines)

# Create a Pygame clock
clock = pygame.time.Clock()

# Pymunk display handler
draw_options = pymunk.pygame_util.DrawOptions(screen)

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Clear the screen
    screen.fill((255, 255, 255))

    # Update physics
    space.step(1/60.0)

    # Draw objects
    space.debug_draw(draw_options)

    # Update the display
    pygame.display.flip()

    # Maintain 60 fps
    clock.tick(60)
