
import pygame
import sys
import random

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Bouncing Ball Simulation")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Ball properties
ball_radius = 20
ball_x = 400
ball_y = 300
ball_speed_x = 0
ball_speed_y = 3
bounce_damping = 0.8
# Main game loop
clock = pygame.time.Clock()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Update ball position
    ball_x += ball_speed_x
    ball_y += ball_speed_y

    # Check for collisions with walls
    if ball_x <= ball_radius or ball_x >= width - ball_radius:
        ball_speed_x = -ball_speed_x
    if ball_y <= ball_radius or ball_y >= height - ball_radius:
        ball_speed_y = -ball_speed_y * bounce_damping
        print(ball_speed_y)

    # Clear the screen
    screen.fill(BLACK)

    # Draw the ball
    pygame.draw.circle(screen, WHITE, (int(ball_x), int(ball_y)), ball_radius)

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    clock.tick(60)


