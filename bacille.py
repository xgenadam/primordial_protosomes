from environment import *
import numpy as np
import random


class Baccilum(Protosome):
    def __init__(self, position, angle, replacement, world_map, angular_velocity=0.0, momentum_vector=(0, 0)):
        vertices = [[-2, -2], [0, -3], [2, -2], [0, 4]]
        color = BLUE
        self.replacement = replacement
        surface_size = np.array([50, 50], dtype=int) * 8
        view_surface = pygame.Surface(surface_size)
        view_surface.fill(BLACK)
        super().__init__(position=position,
                         angle=angle,
                         color=color,
                         hunger=random.randint(50, 150),
                         starvation_factor=0.2,
                         vertices=vertices,
                         angular_velocity=angular_velocity,
                         momentum_vector=np.array(momentum_vector, dtype=float),
                         view_surface=view_surface,
                         world_map=world_map)
        self.sight = None

    def destroy(self):
        super().destroy(replacement=self.replacement)

    def pulse(self):
        self.sight = self.world_map.draw_to_array(self.view_surface, self.body.position, ppm=4,
                                                  direction=self.body.angle)


class Substrate(Map):
    def update_world(self, timestep=None):
        eaten = []
        for food in filter(lambda p: type(p) is Consumable, self.physicals.values()):
            food_collisions = list(filter(lambda c: isinstance(c, Baccilum), food.collisions))
            for baccilum in food_collisions:
                baccilum.hunger += food.hunger_restored/len(food_collisions)
                if food not in eaten:
                    eaten.append(food)

        for danger in filter(lambda p: type(p) is Dangerous, self.physicals.values()):
            danger_collisions = list(filter(lambda c: isinstance(c, Baccilum), danger.collisions))
            for baccilum in danger_collisions:
                baccilum.health += danger.health_restored

        super().update_world(timestep=timestep)

        for food in eaten:
            food.destroy()
        physicals = list(self.physicals.values())
        for physical in physicals:
            physical.pulse()


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

petri_dish = Substrate(corners=corners, wall_color=RED, size=(SCREEN_WIDTH, SCREEN_HEIGHT), ppm=1, timestep=TIME_STEP)


def Food(position, angle, world_map=petri_dish):
    hunger_restored = 10
    vertices = [[-2, -2], [2, -2], [2, 2], [-2, 2]]
    color = WHITE
    return Consumable(world_map=world_map, position=position, angle=angle, hunger_restored=hunger_restored, vertices=vertices, color=color)


def Corpse(position, angle, momentum_vector, angular_velocity, world_map):
    hunger_restored = 20
    vertices = [[-2, -2], [2, -2], [2, 2], [-2, 2]]
    color = GREEN
    return Consumable(world_map=world_map,
                      position=position,
                      angle=angle,
                      hunger_restored=hunger_restored,
                      vertices=vertices,
                      color=color,
                      momentum_vector=momentum_vector,
                      angular_velocity=angular_velocity)

c1 = (np.sqrt(5) - 1)/4
c2 = (np.sqrt(5) + 1)/4

s1 = np.sqrt(10 + 2 * np.sqrt(5))/4
s2 = np.sqrt(10 - 2 * np.sqrt(5))/4

pentagon_vertices = np.array([[0, 1],
                              [s1, c1],
                              [s2, -c2],
                              [-s2, -c2],
                              [-s1, c1]], dtype=float)

pentagon_vertices


def Bomd(position, angle, world_map=petri_dish):
    health_restored = -50
    vertices = pentagon_vertices.tolist()
    color = YELLOW
    return Consumable(world_map=world_map, position=position, angle=angle, health_restored=health_restored,
                      vertices=vertices, color=color)


class Dangerous(Consumable):
    def __init__(self, position, angle):
        world_map = petri_dish
        health_restored = -10
        vertices = (pentagon_vertices * 8).tolist()
        color = RED
        super().__init__(world_map=world_map, position=position, angle=angle, health_restored=health_restored,
                         vertices=vertices, color=color)


map_centre = np.average(corners, axis=0)

creature = Baccilum(position=[random.randint(25, map_x_max - 25), random.randint(25, map_y_max - 25)],
                    angle=random.random() * 2 * np.pi,
                    momentum_vector=[random.randint(-60, 60), random.randint(-60, 60)],
                    replacement=Corpse,
                    world_map=petri_dish)

for x in range(20):
    Baccilum(position=[random.randint(25, map_x_max-25), random.randint(25, map_y_max-25)],
             angle=random.random() * 2 * np.pi,
             momentum_vector=[random.randint(-60, 60), random.randint(-60, 60)],
             replacement=Corpse,
             world_map=petri_dish)


for x in range(10):
    Dangerous(position=[random.randint(25, map_x_max-25), random.randint(25, map_y_max-25)],
              angle=random.random() * 2 * np.pi)


size = np.array([map_x_max, map_y_max], dtype=int)
screen = pygame.display.set_mode(creature.view_surface.get_size())

surface_array = pygame.Surface(size*2)

pygame.display.set_caption("Petri Dish")

clock = pygame.time.Clock()


def main():
    x = 0
    done = False
    while not done:
        # --- Main event loop
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        x += 1

        petri_dish.update_world(timestep=TIME_STEP)
        pygame.surfarray.blit_array(screen, creature.sight)
        pygame.display.flip()

        if x%50 == 0:
            Bomd(position=[random.randint(25, map_x_max-25), random.randint(25, map_y_max-25)],
                   angle=random.random() * 2 * np.pi)

        if x%100 == 0:
            x = 0
            Food(position=[random.randint(25, map_x_max-25), random.randint(25, map_y_max-25)],
                 angle=random.random() * 2 * np.pi)

        clock.tick(TARGET_FPS)

    pygame.quit()

main()

