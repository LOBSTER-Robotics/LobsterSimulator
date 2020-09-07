from lobster_simulator.common.calculations import clip
from lobster_common.vec3 import Vec3
from lobster_simulator.environment.water_surface import WaterSurface
from lobster_simulator.robot import auv
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.common import translation
from lobster_common.constants import *
from lobster_simulator.common.debug_visualization import DebugLine
from lobster_simulator.common.pybullet_api import PybulletAPI, Frame


class Thruster:

    def __init__(self, robot: auv, name, position: Vec3, direction: Vec3,
                 minimum_thrust: float,  # Newton
                 maximum_forward_thrust: float,  # Newton
                 maximum_backward_thrust: float,  # Newton
                 maximum_delta_thrust_per_second: float  # Newton/second
                 ):

        self._name = name
        self._robot = robot
        self._position: Vec3 = position
        self._direction: Vec3 = direction

        self._minimum_thrust = minimum_thrust
        self._maximum_forward_thrust = maximum_forward_thrust
        self._maximum_backward_thrust = maximum_backward_thrust
        self._maximum_delta_thrust_per_second = maximum_delta_thrust_per_second

        self._desired_thrust = 0

        # This is needed to be able to only actually start producing thrust once the minimum thrust is exceeded.
        self._theoretical_thrust = 0

        self._motor_debug_line = DebugLine(self._position, self._position, parentIndex=robot._object_id, color=[0, 0, 1])

    def set_desired_thrust(self, desired_thrust: float) -> None:
        self._desired_thrust = clip(desired_thrust, -self._maximum_backward_thrust, self._maximum_forward_thrust)

    @property
    def current_thrust(self) -> float:
        return self._theoretical_thrust if abs(self._theoretical_thrust) >= self._minimum_thrust else 0.0

    def _update(self, dt: SimulationTime):
        """

        :param dt: Delta time
        """
        if abs(self._desired_thrust - self._theoretical_thrust) <= self._maximum_delta_thrust_per_second * dt.seconds:
            self._theoretical_thrust = self._desired_thrust
        elif self._desired_thrust > self._theoretical_thrust:
            self._theoretical_thrust += self._maximum_delta_thrust_per_second * dt.seconds
        elif self._desired_thrust < self._theoretical_thrust:
            self._theoretical_thrust -= self._maximum_delta_thrust_per_second * dt.seconds

        self._theoretical_thrust = clip(self._theoretical_thrust, -self._maximum_backward_thrust,
                                        self._maximum_forward_thrust)

        world_position = translation.vec3_local_to_world_id(self._robot.object_id, self._position)
        if world_position[Z] > WaterSurface.water_height(world_position[X], world_position[Y]):

            PybulletAPI.applyExternalForce(objectUniqueId=self._robot.object_id,
                                           forceObj=self._direction * self.current_thrust,
                                           posObj=self._position,
                                           frame=Frame.LINK_FRAME)

            debug_line_color = [0, 0, 1]  # Debug line color is blue when the thruster is in the water
        else:
            # Debug line color is red when the thruster is not in the water and can thus give no thrust
            debug_line_color = [1, 0, 0]

        self._motor_debug_line.update(self._position,
                                      self._position
                                      + self._direction * self.current_thrust / 100,
                                      self._robot.object_id,
                                      color=debug_line_color)

    def remove(self):
        self._motor_debug_line.remove()

    @staticmethod
    def new_T200(robot, name, position: Vec3, direction: Vec3):

        # TODO: estimate realistic value for `maximum_delta_thrust_per_second`
        return Thruster(robot, name, position, direction,
                        minimum_thrust=0.196133,
                        maximum_forward_thrust=51.48491,
                        maximum_backward_thrust=40.2073,
                        maximum_delta_thrust_per_second=5000)


