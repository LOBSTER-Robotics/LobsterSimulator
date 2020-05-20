from typing import Dict
import copy
import pybullet as p
import pybullet_data
import json
import time as t

from pkg_resources import resource_stream

from lobster_simulator.robot.Lobster import Lobster
from lobster_simulator.simulation_time import SimulationTime, microseconds_to_seconds, seconds_to_microseconds


class Simulator:

    def __init__(self, time_step: int, config=None, gui=True):
        """
        Simulator
        :param time_step: duration of a step in microseconds
        :param config: config of the robot.
        :param gui: start the PyBullet gui when true
        """
        if config is None:
            with resource_stream('lobster_simulator', 'data/config.json') as f:
                config = json.load(f)
        self.time: SimulationTime = SimulationTime(0)
        self.previous_update_time: SimulationTime = SimulationTime(0)
        self.previous_update_real_time: float = t.perf_counter() # in seconds
        self.time_step : SimulationTime = SimulationTime(initial_microseconds=time_step)
        self.gui = gui

        self.cycle = 0

        self.motor_mapping = {
            'left-front': 0,
            'right-front': 1,
            'top-front': 2,
            'bottom-front': 3,
            'left-side': 4,
            'right-side': 5,
            'top-side': 6,
            'bottom-side': 7
        }

        self.physics_client_id = -1
        if gui:
            self.physics_client_id = p.connect(p.GUI)
            self.simulator_frequency_slider = p.addUserDebugParameter("simulation frequency", 10, 1000,
                                                                      1 / self.time_step.microseconds)
            self.buoyancy_force_slider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 550)

        else:
            self.physics_client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(self.time_step.seconds)
        p.setGravity(0, 0, -10)
        p.loadURDF("plane.urdf", [0, 0, -100])

        with resource_stream('lobster_simulator', 'data/scout-alpha.json') as f:
            lobster_config = json.load(f)

        self.lobster = Lobster(lobster_config)

    def get_pybullet_id(self):
        return self.physics_client_id

    def get_time_in_seconds(self) -> float:
        return self.time.seconds

    def get_position(self):
        return self.lobster.get_position()

    def set_time_step(self, time_step: int):
        """
        Set time step
        :param time_step: in milliseconds
        """
        self.time_step = SimulationTime(time_step)
        p.setTimeStep(self.time_step.seconds)

    def set_rpm_motors(self, rpm_motors):
        self.lobster.set_desired_rpm_motors(rpm_motors)

    def set_thrust_motors(self, pwm_motors: Dict[str, float]):
        for (motor, value) in pwm_motors.items():
            self.lobster.set_desired_thrust_motor(self.motor_mapping[motor], value)

    def step_until(self, time: float):
        """
        Execute steps until time (in seconds) has reached
        :param time:
        :return:
        """
        while (self.time + self.time_step).seconds <= time:
            self.do_step()

    def do_step(self):
        self.time += self.time_step
        if self.gui:
            self.lobster.set_buoyancy(p.readUserDebugParameter(self.buoyancy_force_slider))

            camera_info = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(
                cameraDistance=camera_info[10],
                cameraYaw=camera_info[8],
                cameraPitch=camera_info[9],
                cameraTargetPosition=self.lobster.get_position()
            )

        self.lobster.update(self.time_step, self.time)

        p.stepSimulation()

        self.cycle += 1
        if self.cycle % 50 == 0:
            print("test"+str(
                (self.time - self.previous_update_time).microseconds / seconds_to_microseconds(
                    t.perf_counter() - self.previous_update_real_time)))
            self.previous_update_time = self.time
            self.previous_update_real_time = t.perf_counter()
