import pygame
import numpy as np

from itertools import cycle
from math import copysign
"""
CONSTANTS
"""

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

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


"""
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


def normal_direction(vector):
    return np.matmul(R270, vector)


def normal_direction_AABB(AABB):
    return normal_direction(AABB[1])


def is_potential_intersection(physical_object_1, physical_object_2, check_both=True):
    for edge_1 in physical_object_1.transformed_edges:
        dx, dy = edge_1[1]
        for vertex in physical_object_2.transformed_vertices:
            if 0 < abs(vertex[0] - edge_1[0]) < dx and 0 < abs(vertex[1] - edge_1[2]) < dy:
                return True

    if check_both is True:
        for edge_1 in physical_object_2.transformed_edges:
            dx, dy = edge_1[1]
            for vertex in physical_object_1.transformed_vertices:
                if 0 < abs(vertex[0] - edge_1[0]) < dx and 0 < abs(vertex[1] - edge_1[2]) < dy:
                    return True
    return False


def get_intersection_points(physical_object_1, physicalal_object_2):
    points_of_intersection = []
    for edge_1 in physical_object_1.transformed_edges:
        for edge_2 in physicalal_object_2.transformed_edges:
            qp = edge_1[0] - edge_2[0]
            qp_x_r = qp[0] * edge_2[1][1] - qp[1] * edge_2[1][0]
            if qp_x_r == 0:
                continue

            r_x_s = edge_2[1][0] * edge_1[1][1] - edge_2[1][1] * edge_1[1][0]

            if r_x_s == 0:
                continue

            u = qp_x_r / r_x_s
            if 0.0 > u < 1.0:
                continue

            points_of_intersection.append(edge_1[0] + u * edge_1[1])

    return points_of_intersection


def collision_reaction_direction(physical_object_1, physical_object_2):
    intersection_points = get_intersection_points(physical_object_1, physical_object_2)
    if len(intersection_points) == 2:
        intersection_center = np.average(intersection_points, axis=0)
        rotation_matrix = generate_rotation_matrix(np.pi/4.0)
        normal_vector = np.matmul(rotation_matrix, intersection_points[0] - intersection_center)
        normal_vector = 1/np.sqrt(np.sum(normal_vector**2))
        return normal_vector


def collision_reaction_force(physical_object_1, physical_object_2, timestep):
    """
    this is for collision reactions between 2 BasePolygon objects.
    the physics is a a bit bullshit, but its simple
    :param physical_object_1:
    :param physical_object_2:
    :return: change in momentum and angular velocity for both objects
    """
    # conservation of momentum
    intersection_points = get_intersection_points(physical_object_1, physical_object_2)
    if len(intersection_points) == 0:
        return None
    normal = normal_direction(intersection_points[1] - intersection_points[0])
    theta = np.arctan(normal[0]/normal[1])
    mtm_transfer_vector = np.array([np.cos(theta), np.sin(theta)], dtype=float)  # TODO check this

    # first do linear momentum
    abs_mtm_1_to_2 = np.dot(normal, physical_object_1.momentum_vector)
    lin_mtm_1_to_2 = abs_mtm_1_to_2 * mtm_transfer_vector

    abs_mtm_2_to_1 = np.dot(-normal, physical_object_2.momentum_vector)
    lin_mtm_2_to_1 = abs_mtm_2_to_1 * mtm_transfer_vector

    # now lets look at angular momentum
    intersection_center = np.average(intersection_points, axis=0)
    inter_section_1_vector = intersection_center - physical_object_1.position
    inter_section_2_vector = intersection_center - physical_object_2.position
    theta_1_intersection = np.arctan(inter_section_1_vector[0]/inter_section_1_vector[1]) + np.pi/2
    theta_2_intersection = np.arctan(inter_section_2_vector[0]/inter_section_2_vector[1]) + np.pi/2

    r1_2 = np.linalg.norm(inter_section_1_vector)
    r2_1 = np.linalg.norm(inter_section_2_vector)

    # linear velocity from angular velocity at point of intersection
    amtm_1_2 = r1_2 * physical_object_1.angular_velocity * np.array([np.cos(theta_1_intersection), np.sin(theta_1_intersection)], dtype=float) * physical_object_1.mass/3.0
    amtm_2_1 = r2_1 * physical_object_2.angular_velocity * np.array([np.cos(theta_2_intersection), np.sin(theta_2_intersection)], dtype=float) * physical_object_2.mass/3.0

    F_1_2 = (lin_mtm_1_to_2 + amtm_1_2)/timestep
    F_2_1 = (lin_mtm_2_to_1 + amtm_2_1)/timestep

    reaction_angle_F_1_2 = np.arccos(np.dot(F_1_2, inter_section_1_vector)/(np.linalg.norm(F_1_2) * r1_2))
    reaction_angle_F_2_1 = np.arccos(np.dot(F_2_1, inter_section_2_vector)/(np.linalg.norm(F_2_1) * r2_1))

    lin_array_1_2 = np.array([np.sin(np.pi-theta), np.cos(np.pi-theta)], dtype=float)
    lin_array_2_1 = -1 * lin_array_1_2
    # lin_array_2_1 = np.array([np.sin(reaction_angle_F_2_1), np.cos(reaction_angle_F_2_1)], dtype=float)

    norm_F_1_2 = np.linalg.norm(F_1_2)
    norm_F_2_1 = np.linalg.norm(F_2_1)

    linear_F_1_2 = norm_F_1_2 * np.cos(reaction_angle_F_1_2) * lin_array_1_2
    linear_F_2_1 = norm_F_2_1 * np.cos(reaction_angle_F_2_1) * lin_array_2_1

    ang_F_1_2 = norm_F_1_2 * np.sin(reaction_angle_F_1_2)
    ang_F_2_1 = norm_F_2_1 * np.sin(reaction_angle_F_2_1)

    # return linear force, angular force and radius for obj 1 and obj 2
    return [linear_F_1_2, ang_F_1_2, r1_2], [linear_F_2_1, ang_F_2_1, r2_1]


"""
CLASSES
"""


class BasePolygon(object):
    def __init__(self, color, vertices,
                 world=None, mass=1, friction=0.3, position=(0.0, 0.0), rotation=0.0, pixels_per_meter=100,
                 momentum_vector=np.array([0, 0], dtype=float), angular_velocity=0.0, native_timestep=None):
        self.color = color
        self.vertices = [np.array(vertex, dtype=float) for vertex in vertices]
        self.edges = []
        for index, vertex in enumerate(self.vertices):
            self.edges.append([vertex, self.vertices[(index+1) % len(self.vertices)] - vertex])

        self.momentum_vector = momentum_vector  # x,y, direction
        self.ppm = pixels_per_meter
        self.world = world
        self.mass = mass
        self.friction = friction
        self.position = position
        self.angle = rotation
        self.angular_velocity = angular_velocity
        self.native_timestep = native_timestep

    def apply_force(self, force, timestep):
        self.momentum_vector += force * timestep

    def apply_angular_force(self, angular_force, radius):
        self.angular_velocity += copysign(1, angular_force) * np.sqrt((abs(angular_force) * radius)/self.mass)

    def update_position(self, timestep=None):
        if timestep is None:
            timestep = self.native_timestep
        self.position += self.momentum_vector/self.mass * timestep
        self.angle += self.angular_velocity * timestep

    @property
    def transformed_edges(self):
        rotation_matrix = generate_rotation_matrix(self.angle)
        # absolute_vertices = [self.position - np.matmul(rotation_matrix, vertex) for vertex in self.vertices]
        # return [[a, b - a] for a, b in zip(absolute_vertices, cycle(absolute_vertices[1:]))]
        absolute_edges = [[np.matmul(TOTAL_REFLECTION, np.matmul(rotation_matrix, edge[0])) + self.position,
                           np.matmul(TOTAL_REFLECTION, np.matmul(rotation_matrix, edge[1]))] for edge in self.edges]
        return absolute_edges

    @property
    def edge_bounds(self):
        rotation_matrix = generate_rotation_matrix(self.angle)
        absolute_vertices = [self.position - np.matmul(rotation_matrix, vertex) for vertex in self.vertices]
        return [[a, b] for a, b in zip(absolute_vertices, cycle(absolute_vertices[1:]))]

    @property
    def transformed_vertices(self):
        rotation_matrix = generate_rotation_matrix(self.angle)
        absolute_vertices = [self.position - np.matmul(rotation_matrix, vertex) for vertex in self.vertices]
        return absolute_vertices

    def draw(self, surface, center_pos, direction, ppm=None, relative=True):
        """
        draw this polygon to surface relative to centre position and direction
        """
        if ppm is None:
            ppm = self.ppm

        if relative is True:
            theta = direction - self.angle
        else:
            theta = direction

        rotation_matrix = generate_rotation_matrix(theta)

        relative_vertices = [center_pos - ppm*np.matmul(rotation_matrix, vertex) for vertex in self.vertices]
        # print(relative_vertices)
        pygame.draw.polygon(surface, self.color, relative_vertices)


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
                 hunger=None, health=100, world=None, mass=1, friction=0.3, position=(0.0, 0.0), rotation=0.0,
                 ppm=100, momentum_vector=np.array([0, 0], dtype=float), angular_velocity=0.0, native_timestep=None):
        self.hunger = hunger
        self.health = health
        self.view_surface = view_surface
        super().__init__(color=color,
                         vertices=vertices,
                         world=world,
                         mass=mass,
                         friction=friction,
                         position=position,
                         rotation=rotation,
                         pixels_per_meter=ppm,
                         momentum_vector=momentum_vector,
                         angular_velocity=angular_velocity,
                         native_timestep=native_timestep)

    def pulse(self):
        # self.body.pulse()
        pass


class Wall(BasePolygon):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class Map(object):
    def __init__(self, corners, size, ppm, terrain=None, rotation=0.0, wall_thickness=10, wall_color=GREEN, background=BLACK):
        self.corners = [np.array(corner, dtype=float) for corner in corners]
        self.map_centre = np.average(self.corners, axis=0)
        self.wall_thickness = wall_thickness
        self.wall_color = wall_color
        self.terrain = terrain
        self.background = background
        self.ppm = ppm
        self.physicals = [self.generate_walls()]
        self.rotation = rotation
        self.size = size

    def draw(self, surface, center_pos, direction=0, relative=True, ppm=None):
        if ppm is None:
            ppm = self.ppm
        if relative is True:
            theta = direction - self.rotation
        else:
            theta = direction
        if self.terrain:
            for terrain in self.terrain:
                terrain.draw(surface, center_pos, theta, relative=relative, ppm=ppm)
        if self.physicals:
            for physical in self.physicals:
                physical.draw(surface, center_pos=center_pos, direction=theta, relative=relative, ppm=ppm)

    def generate_walls(self):
        outer_corners = []
        for corner in self.corners:
            outer_corner = [corner[0] + np.sign(corner[0]) * self.wall_thickness, corner[1] + np.sign(corner[1]) * self.wall_thickness]
            outer_corners.append(outer_corner)
        self.corners.append(self.corners[0])
        self.corners.append(outer_corners[0])
        self.corners.extend(outer_corners[-1::-1])
        return Wall(vertices=self.corners, color=self.wall_color)




