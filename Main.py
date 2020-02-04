import sys

import pybullet as p
import pybullet_data

from control.HighLevelController import HighLevelController
from robot.LobsterScout import LobsterScout
from Tools.Constants import *
from Tools.Logger import *


def move_camera_target(target):
    camera_info = p.getDebugVisualizerCamera()

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_info[10],
        cameraYaw=camera_info[8],
        cameraPitch=camera_info[9],
        cameraTargetPosition=target
    )


def main(gui=True, tcp=False):
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    logger = Logger.get_logger()

    if tcp:
        logger.add_tcp_client()

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")

    lobster = LobsterScout(2, 0.2, 0.75, -0.3, 0)

    # Only try to add debug sliders and visualisation when the gui is showing
    if gui:
        forward_thrust_slider = p.addUserDebugParameter("forward thrust", -1, 1, 0)
        desired_pos_sliders = [
            p.addUserDebugParameter("desired x", -100, 100, 0),
            p.addUserDebugParameter("desired y", -100, 100, 0),
            p.addUserDebugParameter("desired z", 0, 100, 1)
        ]
        roll_rate_slider = p.addUserDebugParameter("rate ROLL", -10, 10, 0)
        buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 120)
        totalThrustSlider = p.addUserDebugParameter("Max thrust", 0, 1000, 100)
        debugLine = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), lineWidth=5)

    high_level_controller = HighLevelController()

    desired_location = [0, 0, 2]

    logger.info("x, y, z, desired_x, desired_y, desired_z, pitch_rate, roll_rate, yaw_rate, target_pitch_rate, "
                "target_roll_rate, target_yaw_rate")

    while True:
        qKey = ord('q')
        keys = p.getKeyboardEvents()
        if qKey in keys and keys[qKey] & p.KEY_WAS_TRIGGERED:
            break

        lobster_pos, lobster_orn = lobster.get_position_and_orientation()

        forward_thrust = 0

        # Reading all the debug parameters (only if the gui is showing)
        if gui:
            desired_location = [
                p.readUserDebugParameter(desired_pos_sliders[0]),
                p.readUserDebugParameter(desired_pos_sliders[1]),
                p.readUserDebugParameter(desired_pos_sliders[2])
            ]
            # Add a line from the lobster to the origin
            p.addUserDebugLine(lineFromXYZ=desired_location, lineToXYZ=lobster_pos, replaceItemUniqueId=debugLine,
                               lineWidth=5, lineColorRGB=[1, 0, 0])
            forward_thrust = p.readUserDebugParameter(forward_thrust_slider)

            high_level_controller.set_target_rate(ROLL, p.readUserDebugParameter(roll_rate_slider))

            lobster.set_buoyancy(p.readUserDebugParameter(buoyancyForceSlider))
            lobster.set_max_thrust(p.readUserDebugParameter(totalThrustSlider))

        velocity = p.getBaseVelocity(lobster.id)
        high_level_controller.update(lobster_pos, lobster_orn, velocity, desired_location)

        thrust_values = high_level_controller.motor_outputs

        for i in range(4):
            thrust_values[i] += forward_thrust

        lobster.set_thrust_values(thrust_values)
        lobster.update()

        logger.info(",".join([f'{i:.2f}' for i in
                              list(lobster_pos) +
                              list(high_level_controller.relative_desired_location) +
                              list(high_level_controller.rates) +
                              list(high_level_controller.target_rates)]))

        p.stepSimulation()
        # time.sleep(1. / 240.)

        if gui:
            move_camera_target(lobster.get_position())

    p.disconnect()


if __name__ == '__main__':
    main(sys.argv)
