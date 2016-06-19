# from environment import (Protosome, Map, RED, GREEN, BLUE, BLACK, WHITE, normal_direction, collision_reaction_force)
from environment import *
import numpy as np
import pygame


map_x_max = 700
map_y_max = 500

PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = map_x_max, map_y_max

corners = [[-(map_x_max/2 - 10), -(map_y_max/2 - 10)],
           [-(map_x_max/2 -10), (map_y_max/2 - 10)],
           [(map_x_max/2 -10), (map_y_max/2 - 10)],
           [(map_x_max/2 -10), -(map_y_max/2 - 10)]]


petri_dish = Map(corners=corners, wall_color=RED, size=(SCREEN_WIDTH, SCREEN_HEIGHT), ppm=1, timestep=TIME_STEP)

map_centre = np.average(corners, axis=0)

test_creature_1 = Protosome(color=GREEN, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max/2, map_y_max/2], angle=-np.pi/2,
                            ppm=1, momentum_vector=np.array([0, 0], dtype=float), angular_velocity=-np.pi/32, world_map=petri_dish)

test_creature_2 = Protosome(color=BLUE, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max/2, map_y_max/2 + 70], angle=-np.pi/7,
                            ppm=1, momentum_vector=np.array([0, -5], dtype=float), angular_velocity=np.pi/32, world_map=petri_dish)

test_creature_3 = Protosome(color=RED, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max / 2, map_y_max / 2 - 70],
                            angle=-np.pi / 7, ppm=1, momentum_vector=np.array([0, 5], dtype=float),
                            angular_velocity=-np.pi / 32, world_map=petri_dish)

test_creature_4 = Protosome(color=GREEN+RED, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max / 2 + 80, map_y_max / 2],
                            angle=-np.pi / 2,
                            ppm=1, momentum_vector=np.array([-5, 0], dtype=float), angular_velocity=-np.pi / 32, world_map=petri_dish)

test_creature_5 = Protosome(color=BLUE + RED, vertices=[[-2, -2], [2, -2], [0, 4]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[50, map_y_max / 2],
                            angle=-np.pi / 2+ np.pi/12,
                            ppm=1, momentum_vector=np.array([-5, 0], dtype=float), angular_velocity=0, world_map=petri_dish)

# f = test_creature_5.body.GetWorldVector(localVector=(0.0, -200.0))
# p = test_creature_5.body.GetWorldPoint(localPoint=(0.0, 2.0))
# test_creature_5.body.ApplyForce(f, p, True)

test_creature_5.body.linearVelocity[0] -= -30

pygame.init()

size = (map_x_max*2, map_y_max*2)
screen = pygame.display.set_mode(size)

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
    screen_center = np.array([(map_x_max), (map_y_max)], dtype=int)
    center = np.array(test_creature_1.body.position, dtype=int)
    petri_dish.draw(screen, center, ppm=3, direction=np.pi/4)
    pygame.draw.circle(screen, WHITE, screen_center.astype(int), 5)
    petri_dish.update_world(timestep=TIME_STEP)

    pygame.display.flip()

    clock.tick(60)


pygame.quit()