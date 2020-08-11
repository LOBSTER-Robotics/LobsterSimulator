import math
from typing import Callable

import noise
import pybullet as p

from lobster_common.vec3 import Vec3
from lobster_common.constants import *
from lobster_simulator.common.pybullet_api import PybulletAPI


class Terrain:
    chunks = dict()

    def __init__(self, height_function: Callable[[float, float], float], depth=100):
        self.current_chunk = (0, 0)

        # TODO: Changing the chunk size should not impact the shape of the generated world, however for some reason
        #  it does. If you think you can fix it, go ahead.
        self.chunk_size = 2**7 - 1

        self.points_per_chunk = 2 ** 5
        self.point_spacing = self.chunk_size / (self.points_per_chunk - 1)
        self.render_distance = 3

        self.depth = depth
        self.height_function = height_function

    @staticmethod
    def sine_wave_terrain(depth=100):
        def get_height(x, y):
            return math.sin(x / 20) * 30 + math.sin(y / 30) * 10
        return Terrain(get_height, depth=depth)

    @staticmethod
    def perlin_noise_terrain(depth=100):
        def get_height_perlin(x, y):
            scale = 80
            octaves = 6
            persistence = 0.5
            lacunarity = 2.0
            seed = 1

            height = noise.pnoise2(x / scale, y / scale,
                                   octaves=octaves,
                                   persistence=persistence,
                                   lacunarity=lacunarity,
                                   repeatx=1024,
                                   repeaty=1024,
                                   base=seed)
            height *= 200

            return height

        return Terrain(get_height_perlin, depth=depth)

    def get_height_field(self, chunk_x, chunk_y):
        height_field_data = [0.0] * self.points_per_chunk * self.points_per_chunk
        for j in range(self.points_per_chunk):
            for i in range(self.points_per_chunk):
                world_x = (-chunk_x * self.chunk_size) + self.point_spacing * i
                world_y = (chunk_y * self.chunk_size)  + self.point_spacing * j

                height = self.height_function(world_x, world_y)

                height_field_data[i + j * self.points_per_chunk] = height

        return height_field_data

    def load_chunk(self, chunk_x, chunk_y):
        height_field_data = self.get_height_field(chunk_x, chunk_y)

        middle = (max(height_field_data) + min(height_field_data)) / 2

        terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD,
                                              meshScale=[self.point_spacing, self.point_spacing, 1],
                                              heightfieldTextureScaling=(self.points_per_chunk - 1) / 2,
                                              heightfieldData=height_field_data,
                                              numHeightfieldRows=self.points_per_chunk,
                                              numHeightfieldColumns=self.points_per_chunk)

        terrain = p.createMultiBody(0, terrainShape,
                                    basePosition=Vec3([(self.chunk_size * chunk_x + self.chunk_size / 2),
                                                       (self.chunk_size * chunk_y + self.chunk_size / 2),
                                                       -(middle - self.depth)]).asENU(),
                                    baseOrientation=PybulletAPI.getQuaternionFromEuler(Vec3([0, 0, math.pi])).asENU())

        return terrain

    def update(self, position):
        current_chunk = (int(position[X] // self.chunk_size), int(position[Y] // self.chunk_size))

        if self.current_chunk[X] != current_chunk[X] or self.current_chunk[Y] != current_chunk[Y]:
            new_chunks = dict()

            render_dist_min = self.render_distance//2
            render_dist_max = self.render_distance - render_dist_min

            for i in range(current_chunk[0] - render_dist_min, current_chunk[0] + render_dist_max):
                for j in range(current_chunk[1] - render_dist_min, current_chunk[1] + render_dist_max):
                    if (i, j) not in self.chunks:
                        new_chunks[(i, j)] = self.load_chunk(i, j)
                    else:
                        new_chunks[(i, j)] = self.chunks[(i, j)]

            for key, value in self.chunks.items():
                if key not in new_chunks.keys():
                    PybulletAPI.removeBody(value)

            self.chunks = new_chunks
            self.current_chunk = current_chunk
