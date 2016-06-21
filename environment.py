import pygame
import numpy as np
from math import copysign
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, edgeShape)

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


def normal_direction(vector):
    normal = np.matmul(R270, vector) / np.linalg.norm(vector)
    return normal


def normal_direction_AABB(AABB):
    return normal_direction(AABB[1])


def is_potential_intersection(physical_object_1, physical_object_2):
    # TODO: sort this out
    diff = 5
    for edge_1_start, edge_1_end in physical_object_1.edge_bounds:
        for p2_vertex in physical_object_2.transformed_vertices:
            if edge_1_start[0] + diff >= p2_vertex[0] >= edge_1_end[0] - diff or edge_1_start[0] - diff <= p2_vertex[0] <= edge_1_end[0] + diff:
                if edge_1_start[1] + diff >= p2_vertex[1] >= edge_1_end[1] - diff or edge_1_start[1] - diff <= p2_vertex[1] <= edge_1_end[1] + diff:
                    return True

    for edge_2_start, edge_2_end in physical_object_2.edge_bounds:
        for p1_vertex in physical_object_1.transformed_vertices:
            if edge_2_start[0] + diff >= p1_vertex[0] >= edge_2_end[0] - diff or edge_2_start[0] - diff <= p1_vertex[0] <= edge_2_end[0] + diff:
                if edge_2_start[1] + diff >= p1_vertex[1] >= edge_2_end[1] - diff or edge_2_start[1] - diff <= p1_vertex[1] <= edge_2_end[1] + diff:
                    return True

    return False


def get_vector_angle(v1, v2):
    angle = np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1) * np.linalg.norm(v2)))
    return angle


def get_intersection_points(physical_object_1, physicalal_object_2):
    # use aabb verctors to determine intersections points
    points_of_intersection = []
    for edge_1 in physical_object_1.transformed_edges:
        for edge_2 in physicalal_object_2.transformed_edges:

            A_x, A_y = edge_1[0]
            B_x, B_y = edge_1[1]
            C_x, C_y = edge_2[0]
            D_x, D_y = edge_2[1]

            u_denom = (D_y*B_x - D_x*B_y)
            t_denom = (B_x*D_y - B_y*D_x)

            limit = 0.00001

            if abs(u_denom) < limit or abs(t_denom) < limit:
                continue

            u = (D_y*C_x + D_x*A_y - D_x*C_y - D_y*A_x)/u_denom

            t = (B_x*A_y + B_y*C_x - B_y*A_x - B_x*C_y)/t_denom

            if 0.0 < u <= 1.0 and 0.0 < t <= 1.0:
                point_1 = edge_1[0] + u*edge_1[1]
                point_2 = edge_2[0] + t*edge_2[1]

                point = (point_1 + point_2)/2.0

                points_of_intersection.append(point)

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
    if not (intersection_points and get_intersection_points(physical_object_2, physical_object_1)):
        return None

    if len(intersection_points) != 2:
        return None
    # if np.linalg.norm(intersection_points[1] - intersection_points[0]) < 0.000001:
    #     return None

    normal = normal_direction(intersection_points[1] - intersection_points[0])
    theta = np.arctan(normal[0]/normal[1])
    mtm_transfer_vector = np.array([np.sin(theta), np.cos(theta)], dtype=float)  # TODO check this

    # first do linear momentum
    abs_mtm_1_to_2 = np.dot(normal, physical_object_1.momentum_vector)
    lin_mtm_1_to_2 = abs_mtm_1_to_2 * mtm_transfer_vector

    abs_mtm_2_to_1 = np.dot(-normal, physical_object_2.momentum_vector)
    lin_mtm_2_to_1 = abs_mtm_2_to_1 * mtm_transfer_vector

    intersection_center = np.average(intersection_points, axis=0)
    inter_section_1_vector = intersection_center - physical_object_1.position
    inter_section_2_vector = intersection_center - physical_object_2.position

    # TODO: double check
    if inter_section_1_vector[1] != 0 or inter_section_2_vector[1] != 0:
        theta_1_intersection = np.arctan(inter_section_1_vector[0]/inter_section_1_vector[1]) + np.pi/2
        theta_2_intersection = np.arctan(inter_section_2_vector[0]/inter_section_2_vector[1]) + np.pi/2
    else:
        theta_1_intersection = 0.0
        theta_2_intersection = 0.0

    r1_2 = np.linalg.norm(inter_section_1_vector)
    r2_1 = np.linalg.norm(inter_section_2_vector)

    # linear velocity from angular velocity at point of intersection
    amtm_1_2 = r1_2 * physical_object_1.angular_velocity * np.array([np.cos(theta_1_intersection), np.sin(theta_1_intersection)], dtype=float) * physical_object_1.mass/10.0
    amtm_2_1 = r2_1 * physical_object_2.angular_velocity * np.array([np.cos(theta_2_intersection), np.sin(theta_2_intersection)], dtype=float) * physical_object_2.mass/10.0

    F_1_2 = (lin_mtm_1_to_2 + amtm_1_2)/timestep
    F_2_1 = (lin_mtm_2_to_1 + amtm_2_1)/timestep

    F_1 = F_1_2 - F_2_1
    F_2 = F_2_1 - F_1_2

    norm_F_1_2 = np.linalg.norm(F_1)
    norm_F_2_1 = np.linalg.norm(F_2)

    reaction_angle_F_1 = np.arccos((np.dot(F_1, inter_section_1_vector)/(norm_F_1_2 * r1_2)))
    reaction_angle_F_2 = np.arccos((np.dot(F_2, inter_section_2_vector)/(norm_F_2_1 * r2_1)))

    # print(np.dot(F_1, inter_section_1_vector)/(np.linalg.norm(F_1) * r1_2), np.dot(F_2, inter_section_2_vector)/(np.linalg.norm(F_2) * r2_1))
    # print(np.dot(F_1, inter_section_1_vector), np.dot(F_2, inter_section_2_vector))
    # print(np.dot(F_1, inter_section_1_vector), np.dot(F_2, inter_section_2_vector))
    # print(norm_F_1_2, norm_F_2_1)
    # print(F_1, F_2)
    # print(reaction_angle_F_1, reaction_angle_F_2)
    # print(inter_section_1_vector, inter_section_2_vector)

    lin_array_1_2 = np.array([np.sin(np.pi-theta), np.cos(np.pi-theta)], dtype=float)
    lin_array_2_1 = -1 * lin_array_1_2

    linear_F_1_2 = norm_F_1_2 * np.cos(reaction_angle_F_1) * lin_array_1_2
    linear_F_2_1 = norm_F_2_1 * np.cos(reaction_angle_F_2) * lin_array_2_1

    ang_F_1_2 = norm_F_1_2 * np.sin(reaction_angle_F_1)
    ang_F_2_1 = norm_F_2_1 * np.sin(reaction_angle_F_2)

    # TODO: this is inneficient way of calculating signs of angular component, should work but come up with something more efficient
    # this entire thing is hacky but works
    rot_F_1 = generate_rotation_matrix(get_vector_angle(np.array([0,1], dtype=float), F_1))
    rot_F_2 = generate_rotation_matrix(get_vector_angle(np.array([0,1], dtype=float), F_2))

    unit_F_1 = np.matmul(rot_F_1, F_1/norm_F_1_2)
    unit_F_2 = np.matmul(rot_F_2, F_1/norm_F_1_2)

    unit_inter_section_1 = np.matmul(rot_F_1, (inter_section_1_vector/np.linalg.norm(inter_section_1_vector)))
    unit_inter_section_2 = np.matmul(rot_F_2, (inter_section_2_vector/np.linalg.norm(inter_section_2_vector)))

    if unit_F_1[1] < 0:
        unit_inter_section_1 = np.matmul(R180, inter_section_1_vector)
        # unit_F_1 = np.matmul(R180, unit_F_1)

    if unit_F_2[1] < 0:
        unit_inter_section_2 = np.matmul(R180, inter_section_2_vector)
        # unit_F_2 = np.matmul(R180, unit_F_2)

    # print(unit_F_1, unit_inter_section_1)
    # print(unit_F_2, unit_inter_section_2)

    if 0 < unit_inter_section_1[0]:
        ang_F_1_2 = -ang_F_1_2

    if 0 < unit_inter_section_2[0]:
        ang_F_2_1 = - ang_F_2_1

    return [F_2, ang_F_1_2, r1_2], [F_1, ang_F_2_1, r2_1]


"""
CLASSES
"""


class BasePolygon(object):
    def __init__(self, color, vertices,
                 world_map=None, mass=1.0, density=0.050, friction=0.3, position=(0.0, 0.0), angle=0.0,
                 momentum_vector=np.array([0, 0], dtype=float), angular_velocity=0.0, native_timestep=None,
                 dynamic=True):
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
            world_map.add_physical(self)
            if dynamic is True:
                self.body = self.world.CreateDynamicBody(position=position, angle=angle)
                self.fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density, friction=friction)
            else:
                self.body = world.CreateStaticBody(position=position, angle=angle)

    def apply_force(self, force, timestep):
        momentum = self.momentum_vector + (force * timestep)
        abs_momentum = np.linalg.norm(momentum)
        if abs_momentum > self.max_momentum:
            momentum *= (self.max_momentum / abs_momentum)
        self.momentum_vector = momentum

    def apply_angular_force(self, angular_force, radius):
        angular_velocity = copysign(1, angular_force) * np.sqrt((abs(angular_force) * radius)/self.mass)
        if abs(angular_velocity) > self.max_angular_velocity:
            angular_velocity = copysign(self.max_angular_velocity, angular_velocity)
        self.angular_velocity = angular_velocity

    def apply_reaction(self, linear_force, angular_force, radius, timestep):
        self.apply_force(linear_force, timestep)
        self.apply_angular_force(angular_force, radius)

    def update_position(self, timestep=None):
        if timestep is None:
            timestep = self.native_timestep
        self.position += self.momentum_vector/self.mass * timestep
        self.angle += (self.angular_velocity * timestep) % (2 * np.pi)

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
        # self.body.pulse()
        pass


class Wall(object):
    def __init__(self, vertices, inner_corners, color, position, world):
        # use box2d edge
        self.vertices = vertices
        self.color = color
        self.position = position
        self.world = world
        self.edge_fixtures = []
        self.edge_bodies = []
        for index, corner in enumerate(inner_corners):
            next_corner = inner_corners[(index + 1) % len(inner_corners)]
            position = np.average([corner, next_corner], axis=0)
            body = edgeShape(vertices=[corner, next_corner*2-corner])

            self.edge_bodies.append(body)
            fixture = world.CreateStaticBody(shapes=body, position=corner)
            # fixture.worldCenter = position
            self.edge_fixtures.append(fixture)

    def draw(self, surface, center_pos, direction=None, ppm=1):
        """
        draw this polygon to surface relative to centre position and direction
        """
        # TODO: sort this out!
        surface_size = np.array(surface.get_size(), dtype=float)
        surface_vertical_offset = np.array([0, surface_size[1]], dtype=float)

        offset = (surface_size / 2 - center_pos * ppm)

        # offset = (surface_size / 2 - (center_pos + self.position) * ppm)
        # pygame.draw.circle(surface, WHITE, [int(x) for x in offset], 5)
        # print(self.position)

        # if direction is None:
        #     vertices = [np.matmul(VERTICAL_REFLECTION, (vertex * ppm) + offset) +
        #                 surface_vertical_offset for vertex in self.vertices]
        # else:
        #     rotation_matrix = generate_rotation_matrix(-direction)
        #     vertices = [np.matmul(VERTICAL_REFLECTION, np.matmul(rotation_matrix,
        #                                                          np.array(vertex,
        #                                                                   dtype=float) - center_pos) * ppm + surface_size / 2) + surface_vertical_offset
        #                 for vertex in self.vertices]

        # if direction is not None:
        #     rotation_matrix = generate_rotation_matrix(-direction)
        #     relative_vertices = [center_pos - ppm * np.matmul(rotation_matrix, vertex) for vertex in self.vertices]
        # else:
        #     relative_vertices = [center_pos - ppm * vertex for vertex in self.vertices]
        for body, fixture in zip(self.edge_bodies, self.edge_fixtures):
            surface_vertical_offset = np.array([0, surface_size[1]], dtype=float)
            relative_vertices = [np.matmul(VERTICAL_REFLECTION, (fixture.transform * vertex * ppm) + offset) + surface_vertical_offset for vertex in body.vertices]
            pygame.draw.lines(surface, RED, False, relative_vertices)
            # break

        # pygame.draw.polygon(surface, self.color, relative_vertices)


class Map(object):
    def __init__(self, corners, size, ppm, terrain=None, rotation=0.0, wall_thickness=10, wall_color=GREEN, background=BLACK, timestep=None):
        self.corners = [np.array(corner, dtype=float) for corner in corners]
        self.map_centre = np.average(self.corners, axis=0)
        self.wall_thickness = wall_thickness
        self.wall_color = wall_color
        self.terrain = terrain if terrain is not None else []
        self.background = background
        self.ppm = ppm
        self.physicals = []
        self.rotation = rotation
        self.size = size
        self.timestep = timestep
        self.world = world(gravity=(0, 0), doSleep=True)
        self.walls =self.generate_walls()
        self.terrain += [self.walls]

    def draw(self, surface, center_pos, direction=None, ppm=None):
        surface.fill(self.background)
        # if ppm is None:
        #     ppm = self.ppm
        if not isinstance(center_pos, np.ndarray):
            center_pos = np.array(center_pos, dtype=float)
        if self.terrain:
            for terrain in self.terrain:
                terrain.draw(surface, center_pos, direction=direction, ppm=ppm)
        if self.physicals:
            for physical in self.physicals:
                try:
                    physical.draw(surface, center_pos=center_pos, direction=direction, ppm=ppm)
                except Exception as e:
                    print(e)

    def generate_walls(self):
        outer_corners = []
        inner_corners = self.corners[:]
        for corner in self.corners:
            outer_corner = [corner[0] + np.sign(corner[0]) * self.wall_thickness, corner[1] + np.sign(corner[1]) * self.wall_thickness]
            outer_corners.append(outer_corner)
        self.corners.append(self.corners[0])
        self.corners.append(outer_corners[0])
        self.corners.extend(outer_corners[-1::-1])
        # position = (max(map(lambda corner: corner[0], self.corners)), max(map(lambda corner: corner[1], self.corners)))
        wall_center = np.average(self.corners, axis=0)
        return Wall(vertices=self.corners, inner_corners=inner_corners, color=self.wall_color, position=wall_center, world=self.world)

    def add_physical(self, *physicals):
        for physical in physicals:
            self.physicals.append(physical)

    @property
    def potential_collisions(self):
        collision_list = []
        for index, physical_1 in enumerate(self.physicals):
            for physical_2 in self.physicals[index:]:
                if physical_1 is not physical_2:
                    if is_potential_intersection(physical_1, physical_2):
                        collision_list.append([physical_1, physical_2])

        for terrain in self.terrain:
            for physical in self.physicals:
                if is_potential_intersection(terrain, physical):
                    collision_list.append([physical, terrain])

        return collision_list

    def update_world(self, timestep=None):
        if timestep is None:
            timestep = self.timestep
        self.world.Step(timestep, 10, 10)

