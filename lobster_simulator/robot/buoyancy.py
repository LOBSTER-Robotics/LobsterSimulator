from typing import Optional

import numpy as np

from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.environment.water_surface import WaterSurface
from lobster_simulator.robot import AUV
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class Buoyancy:

    def __init__(self, robot: 'AUV', radius: float, length: float, resolution: Optional[float] = None, visualize: bool = False):
        self._robot: AUV = robot

        self._buoyancy: float = 550

        self.resolution = resolution

        pos = Vec3([0, 0, 0])

        self.visualize = visualize

        self.dots = list()
        self.dot_under_water = list()
        self.test_points = list()

        if resolution:
            x_range = np.arange(-length / 2, length / 2, self.resolution)
            y_range = np.arange(-radius, radius + self.resolution, self.resolution)
            z_range = np.arange(-radius, radius + self.resolution, self.resolution)
        else:
            x_range = range(1)
            y_range = range(1)
            z_range = range(1)

        total_points = len(x_range) * len(y_range) * len(z_range)
        print(f"Total points to check: {total_points}")
        count = 0
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    pos = Vec3([x, y, z])
                    count += 1
                    if count % 1000 == 0:
                        print(f"{int(count/total_points * 100)}%")

                    if np.math.sqrt(y**2 + z**2) < radius and \
                            PybulletAPI.rayTest(pos + Vec3([0, 1, 0]), pos, object_id=self._robot._id)[0] < 1 and \
                            PybulletAPI.rayTest(pos + Vec3([0, 0, 1]), pos, object_id=self._robot._id)[0] < 1 and \
                            PybulletAPI.rayTest(pos + Vec3([0, -1, 0]), pos, object_id=self._robot._id)[0] < 1 and \
                            PybulletAPI.rayTest(pos + Vec3([0, 0, -1]), pos, object_id=self._robot._id)[0] < 1:

                        if self.visualize:
                            self.dot_under_water.append(True)
                            if self.resolution:
                                sphere_size = self.resolution / 4
                            else:
                                sphere_size = 0.05
                            self.dots.append(PybulletAPI.createVisualSphere(sphere_size, [0, 0, 1, 1]))

                        self.test_points.append(Vec3([x, y, z]))

    def _update(self):
        buoyancy_point = Vec3([0, 0, 0])
        buoyancy_force = Vec3([0, 0, -(self._buoyancy)])
        under_water_count = 0

        robot_pos = self._robot.get_position()

        # Only do the computationally intensive calculation of the buoyance point and force when the robot is close to
        #  the surface
        if robot_pos[Z] - 1 < WaterSurface.water_height(robot_pos[X], robot_pos[Y]):
            for i in range(len(self.test_points)):
                position, orientation = self._robot.get_position_and_orientation()

                dot_position = Translation.vec3_local_to_world(position, orientation, self.test_points[i])

                if self.visualize:
                    PybulletAPI.resetBasePositionAndOrientation(self.dots[i], dot_position)

                if dot_position[Z] > WaterSurface.water_height(dot_position[X], dot_position[Y]):
                    under_water_count += 1
                    buoyancy_point += self.test_points[i]
                    if self.visualize and not self.dot_under_water[i]:
                        PybulletAPI.changeVisualShapeColor(self.dots[i], [0, 0, 1, 0.5])
                        self.dot_under_water[i] = True

                else:
                    if self.visualize and self.dot_under_water[i]:
                        PybulletAPI.changeVisualShapeColor(self.dots[i], [1, 0, 0, 0.5])
                        self.dot_under_water[i] = False

            if under_water_count > 0:
                buoyancy_point /= under_water_count

            buoyancy_force = buoyancy_force * under_water_count / len(self.test_points)



        # Apply the buoyancy force
        self._robot.apply_force(buoyancy_point, buoyancy_force, relative_direction=False)
