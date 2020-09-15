from typing import Optional

import numpy as np

from lobster_common.vec3 import Vec3
from lobster_simulator.environment.water_surface import WaterSurface
from lobster_simulator.robot import auv
from lobster_simulator.common import translation
from lobster_common.constants import *
from lobster_simulator.common.pybullet_api import PybulletAPI


class Buoyancy:

    def __init__(self, robot: 'auv', radius: float, length: float, resolution: Optional[float] = None,
                 visualize: bool = False):
        if radius <= 0:
            raise ValueError("Radius should be bigger than zero")
        if length <= 0:
            raise ValueError("Length should be bigger than zero")

        self._radius = radius
        self._length = length

        self._robot: auv = robot

        self._buoyancy: float = 550

        self.resolution = resolution

        pos = Vec3([0, 0, 0])

        self.visualize = visualize

        self.dots = []
        self.dot_under_water = []
        self.test_points = []

        if resolution:
            # Expects that the robot has a cylindrical shape.
            x_range = np.arange(-length / 2, length / 2, self.resolution)
            y_range = np.arange(-radius, radius + self.resolution, self.resolution)
            z_range = np.arange(-radius, radius + self.resolution, self.resolution)
        else:
            x_range = range(1)
            y_range = range(1)
            z_range = range(1)

        total_points = len(x_range) * len(y_range) * len(z_range)
        count = 0
        # Casts rays within a certain radius directed onto the robot object. When the ray hits (or not hits?) the robot the origin of the ray is saved as a test point.
        # Joris could you elaborate here? This is what I thought it would be.
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    pos = Vec3([x, y, z])
                    count += 1
                    if count % 1000 == 0:
                        print(f"{int(count / total_points * 100)}%")

                    if np.math.sqrt(y ** 2 + z ** 2) < radius \
                            and self._check_ray_hits_robot(pos + Vec3([0, 1, 0]), pos) \
                            and self._check_ray_hits_robot(pos + Vec3([0, 0, 1]), pos) \
                            and self._check_ray_hits_robot(pos + Vec3([0, -1, 0]), pos) \
                            and self._check_ray_hits_robot(pos + Vec3([0, 0, -1]), pos):

                        if self.visualize:
                            self.dot_under_water.append(True)
                            if self.resolution:
                                sphere_size = self.resolution / 4
                            else:
                                sphere_size = 0.05
                            self.dots.append(PybulletAPI.createVisualSphere(sphere_size, [0, 0, 1, 1]))

                        self.test_points.append(Vec3([x, y, z]))
        if len(self.test_points) == 0:
            raise NoTestPointsCreated(f"No buoyancy test points where created with robot: {robot}, radius: {radius},"
                                      f" length: {length}"
                                      f" and resolution {resolution} ")

    def _check_ray_hits_robot(self, start_point: Vec3, endpoint: Vec3) -> bool:
        return PybulletAPI.rayTest(start_point, endpoint, object_id=self._robot.object_id)[0] < 1

    def update(self):
        buoyancy_point = Vec3([0, 0, 0])
        buoyancy_force = Vec3([0, 0, -self._buoyancy])
        under_water_count = 0

        position, orientation = self._robot.get_position_and_orientation()

        # Only do the computationally intensive calculation of the buoyancy point and force when the robot is close to
        #  the surface
        # And when there are any points to be used to calculate the buoyancy.
        max_distance_from_pos = max(self._radius, self._length / 2)
        if position[Z] - max_distance_from_pos < WaterSurface.water_height(position[X], position[Y]):
            for i in range(len(self.test_points)):
                dot_position = translation.vec3_local_to_world(position, orientation, self.test_points[i])

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

    def remove(self):
        for dot in self.dots:
            PybulletAPI.removeBody(dot)


class NoTestPointsCreated(RuntimeError):
    """
    No buoyancy test points could be created with the current parameters.
    """
