import time

import pybullet as p
import pybullet_data

from HighLevelController import HighLevelController
from AnimatedPlot import AnimatedPlot
from LobsterScout import LobsterScout


def move_camera_target(target):
    camera_info = p.getDebugVisualizerCamera()

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_info[10],
        cameraYaw=camera_info[8],
        cameraPitch=camera_info[9],
        cameraTargetPosition=target
    )


def main():

    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")

    lobster = LobsterScout(2, 0.2, 0.75, -0.3, 0)

    forward_thrust_slider = p.addUserDebugParameter("forward thrust", -1, 1, 0)

    desired_pos = list()
    desired_pos.append(p.addUserDebugParameter("desired x", -100, 100, 0))
    desired_pos.append(p.addUserDebugParameter("desired y", -100, 100, 0))
    desired_pos.append(p.addUserDebugParameter("desired z", 0, 100, 1))

    debug_line = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), lineWidth=5)

    high_level_controller = HighLevelController()

    while True:
        qKey = ord('q')
        keys = p.getKeyboardEvents()
        if qKey in keys and keys[qKey] & p.KEY_WAS_TRIGGERED:
            break

        lobster_pos, lobster_orn = lobster.get_position_and_orientation()

        desired_location = [
            p.readUserDebugParameter(desired_pos[0]),
            p.readUserDebugParameter(desired_pos[1]),
            p.readUserDebugParameter(desired_pos[2])
        ]

        # Add a line from the lobster to the origin
        p.addUserDebugLine(lineFromXYZ=desired_location, lineToXYZ=lobster_pos, replaceItemUniqueId=debug_line,
                           lineWidth=5, lineColorRGB=[1, 0, 0])

        velocity = p.getBaseVelocity(lobster.id)

        high_level_controller.update(lobster_pos, lobster_orn, velocity, desired_location)

        thrust_values = high_level_controller.motor_outputs

        forward_thrust = p.readUserDebugParameter(forward_thrust_slider)

        for i in range(4):
            thrust_values[i] += forward_thrust

        print(["{0:+0.2f}".format(i) for i in thrust_values], end='')

        lobster.set_thrust_values(thrust_values)
        lobster.update()

        p.stepSimulation()
        time.sleep(1. / 240.)
        move_camera_target(lobster.get_position())

    p.disconnect()


if __name__ == '__main__':
    main()
