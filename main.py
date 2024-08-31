import pygame
import json
from functions import *
import math
import numpy as np

# Reading the test data
f = open('test_data.json')

data = json.load(f)
area_pts = data['area']['coordinates']
area_pts = np.array(area_pts).T

# TODO set these values dynamically based on the input
scale = 50
screen_x = 500
screen_y = 500

# Figuring out screen width and height
area_pts *= scale

min_x = np.min(area_pts[0, :])
max_x = np.max(area_pts[0, :])
min_y = np.min(area_pts[1, :])
max_y = np.max(area_pts[1, :])

# Centering the area in the middle of the screen
area_center = np.mean(area_pts, axis = 1)
print(area_center)

dx = screen_x / 2 - area_center[0]
dy = screen_y / 2 - area_center[1]
t = np.array([dx, dy]).reshape(-1, 1)

area_pts_transformed = area_pts + t

# Creating the Pygame screen
pygame.init()
screen = pygame.display.set_mode((screen_x, screen_y))
clock = pygame.time.Clock()
running = True

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")

    # RENDER YOUR GAME HERE
    area_polygon = list(map(tuple, area_pts_transformed.T))
    pygame.draw.polygon(screen, (0, 0, 255), area_polygon)

    # flip() the display to put your work on screen
    pygame.display.flip()

    clock.tick(60)  # limits FPS to 60

pygame.quit()