import math

from lobster_simulator.common.Calculations import clip
from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.PybulletAPI import PybulletAPI, Frame


class Thruster:

    def __init__(self, pybullet_id: int, name, position: Vec3, direction: Vec3,
                 minimum_thrust: float,  # Newton
                 maximum_forward_thrust: float,  # Newton
                 maximum_backward_thrust: float,  # Newton
                 maximum_delta_thrust_per_second: float  # Newton/second
                 ):

        self._name = name
        self._pybullet_id = pybullet_id
        self._position: Vec3 = position
        self._direction: Vec3 = direction

        self._minimum_thrust = minimum_thrust
        self._maximum_forward_thrust = maximum_forward_thrust
        self._maximum_backward_thrust = maximum_backward_thrust
        self._maximum_delta_thrust_per_second = maximum_delta_thrust_per_second

        self._desired_thrust = 0

        # This is needed to be able to only actually start producing thrust once the minimum thrust is exceeded.
        self._theoretical_thrust = 0

    def set_desired_thrust(self, desired_thrust: float) -> None:
        self._desired_thrust = clip(desired_thrust, -self._maximum_backward_thrust, self._maximum_forward_thrust)

    @property
    def current_thrust(self) -> float:
        return self._theoretical_thrust if abs(self._theoretical_thrust) >= self._minimum_thrust else 0.0

    def _update(self, dt: SimulationTime):
        """

        :param dt: Delta time
        """

        if self._desired_thrust > self._theoretical_thrust:
            self._theoretical_thrust += self._maximum_delta_thrust_per_second * dt.seconds
        elif self._desired_thrust < self._theoretical_thrust:
            self._theoretical_thrust -= self._maximum_delta_thrust_per_second * dt.seconds

        self._theoretical_thrust = clip(self._theoretical_thrust, -self._maximum_backward_thrust, self._maximum_forward_thrust)

        PybulletAPI.applyExternalForce(objectUniqueId=self._pybullet_id,
                                       forceObj=self._direction * self.current_thrust,
                                       posObj=self._position,
                                       frame=Frame.LINK_FRAME)

    @staticmethod
    def new_T200(pybullet_id, name, position: Vec3, direction: Vec3):

        # TODO: estimate realistic value for `maximum_delta_thrust_per_second`
        return Thruster(pybullet_id, name, position, direction,
                        minimum_thrust=0.196133,
                        maximum_forward_thrust=51.48491,
                        maximum_backward_thrust=40.2073,
                        maximum_delta_thrust_per_second=500)


