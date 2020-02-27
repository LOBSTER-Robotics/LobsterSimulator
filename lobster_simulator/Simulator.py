import pybullet as p
import pybullet_data
import json

from pkg_resources import resource_stream

from lobster_simulator.robot.Lobster import Lobster


class Simulator:

    def __init__(self, time_step, config=None, gui=True):
        if config is None:
            with resource_stream('lobster_simulator', 'data/config.json') as f:
                config = json.load(f)
        self.time = 0
        self.time_step = time_step
        self.gui = gui

        self.physics_client_id = -1
        if gui:
            self.physics_client_id = p.connect(p.GUI)
        else:
            self.physics_client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -10)
        p.loadURDF("plane.urdf")

        self.lobster = Lobster(config)

    def get_time(self):
        return self.time

    def get_position(self):
        return self.lobster.get_position()

    def set_time_step(self, time_step):
        self.time_step = time_step
        p.setTimeStep(self.time_step)

    def set_rpm_motors(self, rpm_motors):
        self.lobster.set_desired_rpm_motors(rpm_motors)

    def set_thrust_pwm(self, pwm_motors):
        for i in range(len(pwm_motors)):
            self.lobster.set

    def step_until(self, time):
        while self.time + self.time_step <= time:
            self.do_step()

    def do_step(self):
        self.lobster.update(self.time_step)

        p.stepSimulation()

        if self.gui:
            camera_info = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(
                cameraDistance=camera_info[10],
                cameraYaw=camera_info[8],
                cameraPitch=camera_info[9],
                cameraTargetPosition=self.lobster.get_position()
            )

        self.time += self.time_step
