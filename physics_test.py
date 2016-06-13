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


petri_dish = Map(corners=corners, wall_color=RED, size=(SCREEN_WIDTH, SCREEN_HEIGHT), ppm=1)

map_centre = np.average(corners, axis=0)

test_creature_1 = Protosome(color=GREEN, vertices=[[-20, -20], [20, -20], [0, 40]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max/2, map_y_max/2], angle=-np.pi/2,
                            ppm=1, momentum_vector=np.array([0, 0], dtype=float), angular_velocity=-np.pi/32)

test_creature_2 = Protosome(color=BLUE, vertices=[[-20, -20], [20, -20], [0, 40]], view_surface=None,
                            hunger=None, health=100, mass=1, friction=0.3, position=[map_x_max/2, map_y_max/2 + 70], angle=-np.pi/7,
                            ppm=1, momentum_vector=np.array([0, -5], dtype=float), angular_velocity=np.pi/32)


pygame.init()

size = (map_x_max, map_y_max)
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

    for test_creature in (test_creature_1, test_creature_2):
        test_creature.update_position(TIME_STEP)
        test_creature.draw(screen, center_pos=test_creature.position, direction=test_creature.angle, ppm=1, relative=False)
        petri_dish.draw(screen, center_pos=[map_x_max/2, map_y_max/2], direction=0, relative=True)

        edges = test_creature.transformed_edges

        pygame.draw.line(screen, WHITE, test_creature.position, test_creature.position + test_creature.momentum_vector*10, 5)
        x = 0
        for edge in edges:
            x += 1
            # print(x)
            try:
                edge_center = edge[0] + edge[1]/2
                edge_normal = normal_direction(edge[1]) * 12
                # pygame.draw.circle(screen, WHITE, edge_center.astype(int), 5)
                # pygame.draw.circle(screen, GREEN, edge[0].astype(int), 5)
                pygame.draw.line(screen, RED, edge[0], edge[0] + edge[1], 2)
                pygame.draw.line(screen, GREEN, edge_center, edge_center + edge_normal, 2)
            except Exception as e:
                print(e)
    reaction = collision_reaction_force(test_creature_1, test_creature_2, TIME_STEP)
    # if reaction is not None and not np.isnan(reaction[0][0]).any() and not np.isnan(reaction[1][0]).any():
    if reaction is not None:
        test_creature_1_reaction, test_creature_2_reaction = reaction
        test_creature_1.apply_force(test_creature_1_reaction[0], TIME_STEP)
        test_creature_2.apply_force(test_creature_2_reaction[0], TIME_STEP)

        test_creature_1.apply_angular_force(test_creature_1_reaction[1], test_creature_1_reaction[2])
        test_creature_2.apply_angular_force(test_creature_2_reaction[1], test_creature_2_reaction[2])

    for point in get_intersection_points(test_creature_1, test_creature_2):
        pygame.draw.circle(screen, WHITE, point.astype(int), 5)
    for point in get_intersection_points(test_creature_2, test_creature_1):
        pygame.draw.circle(screen, WHITE, point.astype(int), 5)

    pygame.draw.circle(screen, RED, [int(val) for val in [map_x_max / 2, map_y_max / 2]], 5)

    pygame.display.flip()

    clock.tick(60)


pygame.quit()