import pybullet as p
from pkg_resources import resource_filename

from lobster_common.vec3 import Vec3
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.common.PybulletAPI import PybulletAPI


class WaterSurface:

    @staticmethod
    def water_height(x, y):
        return 0

    def __init__(self, time: SimulationTime):

        PybulletAPI.loadURDF(resource_filename("lobster_simulator", "data/water_surface.urdf"), Vec3([0, 0, 0]))

        self.water_texture = p.loadTexture(resource_filename("lobster_simulator", "data/water_texture.png"))

        p.changeVisualShape(self.water_texture, -1, textureUniqueId=self.water_texture, rgbaColor=[0, 0.3, 1, 0.5])