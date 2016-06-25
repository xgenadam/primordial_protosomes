import pygame
import numpy as np
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, edgeShape, contact, manifold)
from uuid import uuid1
from itertools import chain

"""
CONSTANTS
"""

BLACK = np.array([0, 0, 0], dtype=int)
WHITE = np.array([255, 255, 255], dtype=int)
GREEN = np.array([0, 255, 0], dtype=int)
RED = np.array([255, 0, 0], dtype=int)
BLUE = np.array([0, 0, 255], dtype=int)

R90 = np.array([[np.cos(np.pi/2.0), -np.sin(np.pi/2.0)],
                [np.sin(np.pi/2.0), np.cos(np.pi/2.0)]], dtype=float)

R180 = np.array([[np.cos(np.pi), -np.sin(np.pi)],
                [np.sin(np.pi), np.cos(np.pi)]], dtype=float)

R270 = np.array([[np.cos(-np.pi / 2.0), -np.sin(-np.pi / 2.0)],
                [np.sin(-np.pi / 2.0), np.cos(-np.pi / 2.0)]], dtype=float)

VERTICAL_REFLECTION  = np.array([[1, 0],
                                 [0, -1]], dtype=float)

HORIZONTAL_REFLECTION = np.array([[-1, 0],
                                  [0, 1]], dtype=float)

TOTAL_REFLECTION = np.array([[-1, 0],
                              [0, -1]], dtype=float)


"""linear_F_2_1
FUNCTIONS
"""


def generate_rotation_matrix(theta):
    matrix = np.array([[np.cos(theta), -np.sin(theta)],
                       [np.sin(theta), np.cos(theta)]], dtype=float)
    return matrix


def physical_view(physical_object_1, physical_object_2):
    if physical_object_1.can_see(physical_object_2):
        return True
    else:
        return False


"""
CLASSES
"""


class BasePolygon(object):
    def __init__(self, color, vertices,
                 world_map=None, mass=1.0, density=0.050, friction=0.3, position=(0.0, 0.0), angle=0.0,
                 momentum_vector=np.array([0, 0], dtype=float), angular_velocity=0.0, native_timestep=None,
                 dynamic=True):
        self.uuid = uuid1()
        self.color = color
        self.vertices = [np.array(vertex, dtype=float) for vertex in vertices]
        self.edges = []
        for index, vertex in enumerate(self.vertices):
            self.edges.append([vertex, self.vertices[(index+1) % len(self.vertices)] - vertex])

        self.momentum_vector = momentum_vector  # x,y, direction
        self.world_map = world_map
        self.world = world_map.world
        self.body = None
        self.fixture = None
        self.mass = mass
        self.friction = friction
        self.position = position
        self.angle = angle
        self.angular_velocity = angular_velocity
        self.native_timestep = native_timestep
        if world_map is not None:
            if dynamic is True:
                self.body = self.world.CreateDynamicBody(position=position, angle=angle)
                self.fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density, friction=friction)
            else:
                self.body = world.CreateStaticBody(position=position, angle=angle)
            self.body.userData = self
            world_map.add_physical(self)

    # with box2d integration these properties need sorting
    @property
    def transformed_edges(self):
        rotation_matrix = generate_rotation_matrix(self.angle)
        absolute_edges = [[np.matmul(TOTAL_REFLECTION, np.matmul(rotation_matrix, edge[0])) + self.position,
                           np.matmul(TOTAL_REFLECTION, np.matmul(rotation_matrix, edge[1]))] for edge in self.edges]

        return absolute_edges

    @property
    def edge_bounds(self):
        transformed_vertices = self.transformed_vertices
        bounds = []
        for index, vertex in enumerate(transformed_vertices):
            bounds.append([vertex, transformed_vertices[(index + 1) % len(transformed_vertices)]])
        return bounds

    @property
    def transformed_vertices(self):
        rotation_matrix = generate_rotation_matrix(self.angle)
        absolute_vertices = [self.position - np.matmul(rotation_matrix, vertex) for vertex in self.vertices]
        return absolute_vertices

    def draw(self, surface, center_pos, direction=None, ppm=1):
        """
        draw this polygon to surface relative to centre position and direction
        """

        surface_size = np.array(surface.get_size(), dtype=float)
        surface_vertical_offset = np.array([0, surface_size[1]], dtype=float)

        if direction is None:
            offset = (surface_size / 2 - center_pos * ppm)
            vertices = [np.matmul(VERTICAL_REFLECTION, (self.body.transform * vertex * ppm) + offset) +
                        surface_vertical_offset for vertex in self.fixture.shape.vertices]
        else:
            rotation_matrix = generate_rotation_matrix(-direction)
            vertices = [np.matmul(VERTICAL_REFLECTION, np.matmul(rotation_matrix,
                                                                 np.array(self.body.transform * vertex, dtype=float) - center_pos) * ppm + surface_size/2) + surface_vertical_offset
                        for vertex in self.fixture.shape.vertices]

        pygame.draw.polygon(surface, self.color, vertices)

    def destroy(self, replacement=None):
        if replacement is not None:
            self.world_map.add_physical(replacement(position=self.body.position, angle=self.body.angle))
        self.world.DestroyBody(self.body)
        return self.world_map.physicals.pop(self.uuid)


class ViewSurface(pygame.Surface):
    def __init__(self, height, width, *args, **kwargs):
        self.height = height
        self.width = width
        # self.surface = pygame.Surface((self.width, self.height))
        self.super().__init__((self.height, self.width), *args, **kwargs)

    def get_view(self, return_array=True):
        pass

    def set_view(self):
        pass


class Protosome(BasePolygon):
    def __init__(self, color, vertices, view_surface,
                 hunger=None, health=100, world_map=None, mass=1, friction=0.3, position=(0.0, 0.0), angle=0.0, angular_velocity=0.0):
        self.hunger = hunger
        self.health = health
        self.view_surface = view_surface
        super().__init__(color=color,
                         vertices=vertices,
                         world_map=world_map,
                         mass=mass,
                         friction=friction,
                         position=position,
                         angle=angle,
                         angular_velocity=angular_velocity,)

    def pulse(self):
        pass

    def destroy(self, replacement=None):
        super().destroy(replacement)


class Wall(object):
    def __init__(self, vertices, inner_corners, color, position, world):
        # use box2d edge
        self.uuid = uuid1()
        self.vertices = vertices
        self.color = color
        self.position = position
        self.world = world
        self.edge_fixtures = []
        self.edge_bodies = []
        for index, corner in enumerate(inner_corners):
            next_corner = inner_corners[(index + 1) % len(inner_corners)]
            fixture = edgeShape(vertices=[corner, next_corner*2-corner])

            self.edge_fixtures.append(fixture)
            body = world.CreateStaticBody(shapes=fixture, position=corner)
            body.userData = self
            self.edge_bodies.append(body)

    def draw(self, surface, center_pos, direction=None, ppm=1):
        """
        draw this polygon to surface relative to centre position and direction
        """
        # TODO: sort this out!
        surface_size = np.array(surface.get_size(), dtype=float)
        surface_vertical_offset = np.array([0, surface_size[1]], dtype=float)

        offset = (surface_size / 2 - center_pos * ppm)

        for body, fixture in zip(self.edge_bodies, self.edge_fixtures):
            if direction is None:
                surface_vertical_offset = np.array([0, surface_size[1]], dtype=float)
                relative_vertices = [np.matmul(VERTICAL_REFLECTION, (body.transform * vertex * ppm) + offset) + surface_vertical_offset for vertex in fixture.vertices]
            else:
                rotation_matrix = generate_rotation_matrix(-direction)
                relative_vertices = [np.matmul(VERTICAL_REFLECTION, np.matmul(rotation_matrix,
                                                                              np.array(body.transform * vertex,
                                                                                       dtype=float) - center_pos) *
                                               ppm + surface_size / 2) + surface_vertical_offset for vertex in fixture.vertices]
            pygame.draw.lines(surface, self.color, False, relative_vertices)

    def get_collisions(self):
        collisions = []
        for body in self.edge_bodies:
            collisions.extend(map(lambda contact: contact.other.userData, body.contacts))
        return collisions

    def destroy(self, *args, **kwargs):
        pass


class Map(object):
    def __init__(self, corners, size, ppm, terrain=None, rotation=0.0, wall_thickness=10, wall_color=GREEN, background=BLACK, timestep=None):
        self.corners = [np.array(corner, dtype=float) for corner in corners]
        self.map_centre = np.average(self.corners, axis=0)
        self.wall_thickness = wall_thickness
        self.wall_color = wall_color
        self.terrain = {}  # terrain if terrain is not None else []
        self.background = background
        self.ppm = ppm
        self.physicals = {}
        self.rotation = rotation
        self.size = size
        self.timestep = timestep
        self.world = world(gravity=(0, 0), doSleep=True)
        self.walls = self.generate_walls()
        self.terrain[self.walls.uuid] = self.walls
        self.physical_bodies = {}

    def draw(self, surface, center_pos, direction=None, ppm=None):
        surface.fill(self.background)
        if not isinstance(center_pos, np.ndarray):
            center_pos = np.array(center_pos, dtype=float)
        if self.terrain:
            for uuid, terrain in self.terrain.items():
                terrain.draw(surface, center_pos, direction=direction, ppm=ppm)
        if self.physicals:
            for uuid, physical in self.physicals.items():
                try:
                    physical.draw(surface, center_pos=center_pos, direction=direction, ppm=ppm)
                except Exception as e:
                    print(e)

    def draw_to_array(self, surface, center_pos, direction=None, ppm=None):
        self.draw(surface, center_pos, direction, ppm)
        return pygame.surfarray.array3d(surface)

    def generate_walls(self):
        outer_corners = []
        inner_corners = self.corners[:]
        for corner in self.corners:
            outer_corner = [corner[0] + np.sign(corner[0]) * self.wall_thickness, corner[1] + np.sign(corner[1]) * self.wall_thickness]
            outer_corners.append(outer_corner)
        self.corners.append(self.corners[0])
        self.corners.append(outer_corners[0])
        self.corners.extend(outer_corners[-1::-1])
        wall_center = np.average(self.corners, axis=0)
        return Wall(vertices=self.corners, inner_corners=inner_corners, color=self.wall_color, position=wall_center, world=self.world)

    def add_physical(self, *physicals):
        for physical in physicals:
            self.physicals[physical.uuid] = physical

    def destroy_physicals(self,  physicals, replacement=None):
        if not physicals:
            return
        for physical in physicals:
            physical.destroy(replacement)

    @property
    def collisions(self):
        collision_dict = {}
        for uuid, physical in self.physicals.items():
            collision_dict[physical.uuid] = map(lambda contact: contact.other.userData, physical.body.contacts)

        collision_dict[self.walls.uuid] = self.walls.get_collisions()

        return collision_dict

    def update_world(self, timestep=None):
        if timestep is None:
            timestep = self.timestep
        self.world.Step(timestep, 10, 10)

