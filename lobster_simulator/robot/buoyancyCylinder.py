import math

from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.environment.water_surface import WaterSurface
from lobster_simulator.robot import AUV
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class BuoyancyCylinder:

    def __init__(self, robot: 'AUV', radius: float, length: float, number_test_points_x: int):
        self.robot: AUV = robot

        self._buoyancy: float = 550

        self.number_test_points_x = number_test_points_x

        self.dots = list()
        self.test_points = list()
        for i in range(number_test_points_x):
            x = -length/2 + (length*i)/(number_test_points_x-1)

            self.dots.append(PybulletAPI.createVisualSphere(radius, [0, 0, 1, 0.5]))
            self.test_points.append(Vec3([x, 0, 0]))

    def _update(self):
        # PybulletAPI.resetBasePositionAndOrientation(self.cylinder, self.robot.get_position(),
        #                                             self.robot.get_orientation())

        buoyancy_point = Vec3([0, 0, 0])
        under_water_count = 0

        for i in range(self.number_test_points_x):
            position, orientation = self.robot.get_position_and_orientation()

            dot_position = Translation.vec3_local_to_world(position, orientation, self.test_points[i])

            PybulletAPI.resetBasePositionAndOrientation(self.dots[i], dot_position)

            if dot_position[Z] > WaterSurface.height_function(dot_position[X], dot_position[Y]):
                under_water_count += 1
                buoyancy_point += self.test_points[i]
                # PybulletAPI.changeVisualShapeColor(self.dots[i], [0, 0, 1, 0.5])
            else:
                pass
                # PybulletAPI.changeVisualShapeColor(self.dots[i], [1, 0, 2, 0.5])

        if under_water_count > 0:
            buoyancy_point /= under_water_count

        print(under_water_count, buoyancy_point)

        # Apply the buoyancy force
        self.robot.apply_force(buoyancy_point, Vec3([0, 0, -(self._buoyancy * under_water_count/self.number_test_points_x)]), relative_direction=False)