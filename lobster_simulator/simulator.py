import copy

import json
import time as t
from typing import Optional

from pkg_resources import resource_stream, resource_filename

from lobster_common.vec3 import Vec3
from lobster_simulator.environment.water_surface import WaterSurface
from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.robot.auv import AUV
from lobster_simulator.common.simulation_time import SimulationTime
from enum import Enum, auto


class Models(Enum):
    SCOUT_ALPHA = auto()
    PTV = auto()


class Simulator:

    def __init__(self, time_step: int, config=None, gui=True):
        """
        Simulator
        :param time_step: duration of a step in microseconds
        :param config: config of the robot.
        :param gui: start the PyBullet gui when true
        """
        with resource_stream('lobster_simulator', 'data/config.json') as f:
            base_config = json.load(f)

        if config is not None:
            base_config.update(config)

        config = base_config

        self.rotate_camera_with_robot = bool(config['rotate_camera_with_robot'])

        self._time: SimulationTime = SimulationTime(0)
        self._previous_update_time: SimulationTime = SimulationTime(0)
        self._previous_update_real_time: float = t.perf_counter()  # in seconds
        self._time_step: SimulationTime = SimulationTime(initial_microseconds=time_step)

        self._cycle = 0

        PybulletAPI.initialize(self._time_step, gui)

        self.water_surface = WaterSurface(self._time)

        self._simulator_frequency_slider = PybulletAPI.addUserDebugParameter("simulation frequency", 10, 1000,
                                                                             1 / self._time_step.microseconds)
        self._buoyancy_force_slider = PybulletAPI.addUserDebugParameter("buoyancyForce", 0, 1000, 550)

        self._model = None
        self.robot_config = None

        self._camera_position = Vec3([0, 0, 0])
        self._robot: Optional[AUV] = None

    def add_ocean_floor(self, depth=100):
        id = PybulletAPI.loadURDF(resource_filename("lobster_simulator", "data/plane1000.urdf"),
                                  base_position=Vec3((0, 0, depth)))

        texture = PybulletAPI.loadTexture(resource_filename("lobster_simulator", "data/checker_blue.png"))
        PybulletAPI.changeVisualShape(id, texture, rgbaColor=[1,1,1,1])

    def get_time_in_seconds(self) -> float:
        return self._time.seconds

    def set_time_step(self, time_step_microseconds: int) -> None:
        """
        Sets the size of the time steps the simulator makes
        :param time_step_microseconds: Time step in microseconds
        """

        self._time_step = SimulationTime(time_step_microseconds)
        PybulletAPI.setTimeStep(self._time_step)

    def do_step(self):
        """Progresses the simulation by exactly one time step."""

        self._time.add_time_step(self._time_step.microseconds)

        self.update_camera_position()

        self._robot.update(self._time_step, self._time)

        PybulletAPI.stepSimulation()

        self._cycle += 1
        if self._cycle % 50 == 0:
            self._previous_update_time = copy.copy(self._time)
            self._previous_update_real_time = t.perf_counter()

    def step_until(self, time: float):
        """
        Execute steps until time (in seconds) has reached. The given time will never be exceeded, but could be slightly
        less than the specified time (at most 1 time step off).
        :param time: Time (in seconds) to which the simulator should run
        """
        while (self._time + self._time_step).seconds <= time:
            self.do_step()

    def update_camera_position(self):
        smoothing = 0.95
        self._camera_position = smoothing * self._camera_position + (1 - smoothing) * self._robot.get_position()
        if self.rotate_camera_with_robot:
            PybulletAPI.moveCameraToPosition(self._camera_position, self._robot.get_orientation())
        else:
            PybulletAPI.moveCameraToPosition(self._camera_position)

    @property
    def robot(self) -> AUV:
        """
        Gets the current instance of the robot.
        :return: Robot instance
        """
        return self._robot

    def create_robot(self, model: Models = Models.SCOUT_ALPHA, **kwargs) -> AUV:
        """
        Creates a new robot based on the given model. If a robot already exists it is overwritten.
        :param model: Model of the robot. (Scout-alpha, PTV)
        :return: Robot instance
        """
        if self._robot:
            self._robot.remove()

        if model == Models.SCOUT_ALPHA:
            model_config = 'scout-alpha.json'
        else:
            model_config = 'ptv.json'

        with resource_stream('lobster_simulator', f'data/{model_config}') as f:
            self.robot_config = json.load(f)

        # Add extra arguments to the robot config
        self.robot_config.update(kwargs)

        self._robot = AUV(self._time, self.robot_config)

        return self._robot

    def reset_robot(self):
        """
        Resets the robot using the same configuration.
        """
        self.create_robot(self._model, **self.robot_config)

    def shutdown(self):
        """
        Shuts down the pybullet Simulator. (This is needed when running multiple tests with the simulator because then
        it doesn't automatically closes in between tests)
        :return:
        """
        PybulletAPI.disconnect()
