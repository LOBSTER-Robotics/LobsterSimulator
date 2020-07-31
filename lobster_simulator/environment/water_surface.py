import math

import pybullet as p
from pkg_resources import resource_filename

from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class WaterSurface:

    @staticmethod
    def height_function(x, y):
        return math.sin(x / 10 + WaterSurface.time.seconds*5) * 2
        # return 0


    def __init__(self, time: SimulationTime):

        self.point_spacing = 1
        self.size = 50

        WaterSurface.time = time

        self.number_of_points = int((self.size / self.point_spacing) + 1)

        self.chunk_size = (self.number_of_points - 1) * self.point_spacing

        height_field_data = self.get_height_field(0, 0)
        middle = (max(height_field_data) + min(height_field_data)) / 2
        self.surface_shape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD,
                                                    meshScale=[self.point_spacing, self.point_spacing, 1],
                                                    heightfieldTextureScaling=(self.number_of_points - 1) / 2,
                                                    collisionFramePosition=[0, 0, 100],
                                                    heightfieldData=height_field_data,
                                                    numHeightfieldRows=self.number_of_points,
                                                    numHeightfieldColumns=self.number_of_points)

        self.terrain = p.createMultiBody(0, self.surface_shape,
                                         basePosition=Vec3([0, 0,
                                                            -(middle - 0)]).asENU(),
                                         baseOrientation=PybulletAPI.getQuaternionFromEuler(
                                             Vec3([0, 0, math.pi])).asENU())

        print(p.getVisualShapeData(self.surface_shape))

        # self.water_texture = p.loadTexture(resource_filename("lobster_simulator", "data/water_texture.png"))
        self.water_texture = p.loadTexture("heightmaps/gimp_overlay_out.png")

        p.changeVisualShape(self.water_texture, -1, textureUniqueId=self.water_texture, rgbaColor=[0, 0.3, 1, 0.7])

    def update(self, time: SimulationTime):
        WaterSurface.time = time
        height_field_data = self.get_height_field(0, 0)
        middle = (max(height_field_data) + min(height_field_data)) / 2
        # self.surface_shape = p.createCollisionShape(
        #     shapeType=p.GEOM_HEIGHTFIELD,
        #     meshScale=[self.point_spacing, self.point_spacing, 1],
        #     heightfieldTextureScaling=(self.number_of_points - 1) / 2,
        #     collisionFramePosition=[0, 0, 100],
        #     heightfieldData=height_field_data,
        #     numHeightfieldRows=self.number_of_points,
        #     numHeightfieldColumns=self.number_of_points,
        #     replaceHeightfieldIndex=self.surface_shape
        # )

        # PybulletAPI.resetBasePositionAndOrientation(self.terrain, Vec3([0, 0, -(middle - 0)]))
        # p.changeVisualShape(self.surface_shape, -1, textureUniqueId=self.water_texture)

    def get_height_field(self, chunk_x, chunk_y):
        height_field_data = [0.0] * self.number_of_points * self.number_of_points
        for j in range(self.number_of_points):
            for i in range(self.number_of_points):
                world_x = (-chunk_x * self.chunk_size) + self.point_spacing * i
                world_y = (chunk_y * self.chunk_size) + self.point_spacing * j

                height = self.height_function(world_x, world_y)

                height_field_data[i + j * self.number_of_points] = height

        return height_field_data
