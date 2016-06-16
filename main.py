from environment import (Protosome, Map, RED, GREEN, BLUE, BLACK, WHITE, normal_direction)
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

test_creature = Protosome(color=BLUE, vertices=[[-20, -20], [20, -20], [0, 40]], view_surface=None,
                          hunger=None, health=100, density=1, friction=0.3, position=[map_x_max/2, map_y_max/2], angle=0.0,
                          ppm=1, momentum_vector=np.array([0, 0], dtype=float))


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
    test_creature.angle = theta
    test_creature.draw(screen, center_pos=test_creature.position, direction=test_creature.angle, ppm=1, relative=False)
    petri_dish.draw(screen, center_pos=[map_x_max/2, map_y_max/2], direction=0, relative=True)

    edges = test_creature.transformed_edges

    # edge_center =[int(val) for val in edges[2][0] + edges[2][1]/2.0]
    #
    # pygame.draw.circle(screen, WHITE, edge_center, 5)
    # edge_normal = normal_direction(edges[2])
    # # pygame.draw.line(screen, GREEN, edge_center, edge_center + edge_normal, 2)

    pygame.draw.circle(screen, RED, [int(val) for val in test_creature.position], 5)
    x = 0
    for edge in edges:
        x += 1
        # print(x)
        try:
            edge_center = edge[0] + edge[1]/2
            edge_normal = normal_direction(edge[1])
            pygame.draw.circle(screen, WHITE, edge_center.astype(int), 5)
            pygame.draw.circle(screen, GREEN, edge[0].astype(int), 5)
            pygame.draw.line(screen, RED, edge[0], edge[0] + edge[1], 2)
            pygame.draw.line(screen, GREEN, edge_center, edge_center + edge_normal, 2)
        except Exception as e:
            print(e)
            print(edge_normal)

    # for vertex in test_creature.transformed_vertices:
    #     pygame.draw.circle(screen, WHITE, vertex.astype(int), 5)
    theta += d_theta
    test_creature.angle = theta

    pygame.display.flip()

    # done = True

    clock.tick(60)


pygame.quit()