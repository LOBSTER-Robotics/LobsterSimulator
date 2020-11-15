from pkg_resources import resource_filename

from lobster_common.vec3 import Vec3
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.common.pybullet_api import PybulletAPI


class WaterSurface:

    @staticmethod
    def water_height(x, y):
        return 0

    def __init__(self, time: SimulationTime):
        water_id = PybulletAPI.loadURDF(resource_filename("lobster_simulator", "data/water_surface.urdf"), Vec3([0, 0, 0]))

        self.water_texture = PybulletAPI.loadTexture(resource_filename("lobster_simulator", "data/water_texture.png"))

        PybulletAPI.changeVisualShape(water_id, textureUniqueId=self.water_texture, rgbaColor=[0, 0.3, 1, 0.5])
