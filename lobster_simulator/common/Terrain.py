import math

import pybullet as p

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.common.perlin import perlin_array
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.PybulletAPI import PybulletAPI
import numpy as np


class Terrain:
    chunks = dict()

    points_per_chunk = 2 ** 7
    point_spacing = 0.5

    def __init__(self, depth=100):
        self.current_chunk = (0, 0)

        self.depth = depth

    @property
    def chunk_size(self):
        return (self.points_per_chunk - 1) * self.point_spacing

    def get_height(self, x, y):
        return math.sin(x / 100) * 100 + math.sin(y / 30) * 10

    def load_chunk(self, chunk_x, chunk_y):

        if False:
            height_field_data = [0.0] * self.points_per_chunk * self.points_per_chunk
            for j in range(self.points_per_chunk):
                for i in range(self.points_per_chunk):
                    world_x = (-chunk_x * (self.points_per_chunk - 1) + i) * self.point_spacing
                    world_y = (chunk_y * (self.points_per_chunk - 1) + j) * self.point_spacing

                    height = self.get_height(world_x, world_y)
                    height_field_data[i + j * self.points_per_chunk] = height
        else:
            height_field_data = perlin_array((self.points_per_chunk, self.points_per_chunk),
                                             scale=50/self.point_spacing,
                                             offset=((self.points_per_chunk - 1) * chunk_y,
                                                     (self.points_per_chunk - 1) * -chunk_x)).flatten() \
                                * 100

        middle = (max(height_field_data) + min(height_field_data)) / 2

        print(middle)

        terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD,
                                              meshScale=[self.point_spacing, self.point_spacing,
                                                         1],
                                              heightfieldTextureScaling=(self.points_per_chunk - 1),
                                              heightfieldData=height_field_data,
                                              numHeightfieldRows=self.points_per_chunk,
                                              numHeightfieldColumns=self.points_per_chunk)

        terrain = p.createMultiBody(0, terrainShape,
                                    basePosition=Vec3([(self.chunk_size * chunk_x + self.chunk_size / 2),
                                                       (self.chunk_size * chunk_y + self.chunk_size / 2),
                                                       -(middle - self.depth)]).asENU(),
                                    baseOrientation=PybulletAPI.getQuaternionFromEuler(Vec3([0, 0, math.pi])).asENU())

        return terrain

    def remove_chunk(self, id):
        print("Removing", id)
        p.removeBody(id)

    def update(self, position):
        current_chunk = (int(position[X] // self.chunk_size), int(position[Y] // self.chunk_size))

        if self.current_chunk[X] != current_chunk[X] or self.current_chunk[Y] != current_chunk[Y]:
            print("Updated chunk", current_chunk, self.current_chunk)

            new_chunks = dict()

            for i in range(current_chunk[0] - 1, current_chunk[0] + 2):
                for j in range(current_chunk[1] - 1, current_chunk[1] + 2):
                    # i, j = current_chunk[0], current_chunk[1]
                    if (i, j) not in self.chunks:
                        new_chunks[(i, j)] = self.load_chunk(i, j)
                    else:
                        new_chunks[(i, j)] = self.chunks[(i, j)]

            for key, value in self.chunks.items():
                if key not in new_chunks.keys():
                    print(f"{key} not in {new_chunks.keys()} so removing")
                    self.remove_chunk(value)

            self.chunks = new_chunks
            self.current_chunk = current_chunk
