import random
from environment import *
import numpy as np
import pygame


map_x_max = 700
map_y_max = 500

PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = map_x_max, map_y_max

corners = [[-(0 - 10), -(0- 10)],
           [-(0 -10), (map_y_max/2 - 10)],
           [(map_x_max/2 -10), (map_y_max/2 - 10)],
           [(map_x_max/2 -10), -(0 - 10)]]


petri_dish = Map(corners=corners, wall_color=RED, size=(SCREEN_WIDTH, SCREEN_HEIGHT), ppm=1, timestep=TIME_STEP)

map_centre = np.average(corners, axis=0)

test_creature_1 = Protosome(color=GREEN, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max/2, map_y_max/2], angle=np.pi/2,
                            angular_velocity=-np.pi/32, world_map=petri_dish)

test_creature_2 = Protosome(color=BLUE, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max/2, map_y_max/2 + 70], angle=-np.pi/7,
                            angular_velocity=np.pi/32, world_map=petri_dish)

test_creature_3 = Protosome(color=RED, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max / 2, map_y_max / 2 - 70],
                            angle=-np.pi / 7, angular_velocity=-np.pi / 32, world_map=petri_dish)

test_creature_4 = Protosome(color=GREEN+RED, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max / 2 + 80, map_y_max / 2],
                            angle=-np.pi / 2, angular_velocity=-np.pi / 32, world_map=petri_dish)

test_creature_5 = Protosome(color=BLUE + RED, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[50, map_y_max / 2],
                            angle=-np.pi / 2, angular_velocity=0, world_map=petri_dish)

for x in range(500):
    creature = Protosome(color=BLUE + GREEN, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                         hunger=None, health=100, mass=1, friction=0.3, position=[random.randint(25, map_x_max-25), random.randint(25, map_y_max-25)],
                         angle=random.random() * 2 * np.pi, angular_velocity=0, world_map=petri_dish)
    creature.body.linearVelocity[0] += random.randint(-60, 60)
    creature.body.linearVelocity[1] += random.randint(-60, 60)

test_creature_5.body.linearVelocity[0] -= -30

pygame.init()

size = (map_x_max, map_y_max)
screen = pygame.display.set_mode(size)

surface_array = pygame.Surface(size)

pygame.display.set_caption("Petri Dish")

done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()
theta = 0.0
d_theta = np.pi/96
# -------- Main Program Loop -----------
while not done:
    # --- Main event loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True


    screen.fill(BLACK)
    screen_center = np.array([(map_x_max/2), (map_y_max/2)], dtype=int)
    center = np.array(test_creature_1.body.position, dtype=int)
    array = petri_dish.draw_to_array(surface_array, center, ppm=4, direction=test_creature_1.body.angle)
    pygame.surfarray.blit_array(screen, array)

    petri_dish.update_world(timestep=TIME_STEP)

    pygame.display.flip()

    clock.tick(60)


pygame.quit()