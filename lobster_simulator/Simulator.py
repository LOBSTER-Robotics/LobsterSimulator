import pybullet as p
import pybullet_data

from lobster_simulator.robot.Lobster import Lobster


class Simulator:

    def __init__(self, time_step, config, gui=True):
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

        self.lobster = Lobster(config['length'],
                               config['diameter'],
                               config['arm_length'],
                               config['arm_position_from_center'],
                               config['center_of_mass'],
                               config['inner_motor_distance_from_center'])

    def get_time(self):
        return self.time

    def get_position(self):
        return self.lobster.get_position()

    def set_time_step(self, time_step):
        self.time_step = time_step
        p.setTimeStep(self.time_step)

    def set_rpm_motors(self, rpm_motors):
        pass

    def do_step(self):

        self.lobster.update()

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
