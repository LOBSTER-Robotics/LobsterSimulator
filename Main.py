import time
import sys
import socket

import pybullet as p
import pybullet_data

from HighLevelController import HighLevelController
from LobsterScout import LobsterScout
from Constants import *


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
        physics_client = p.connect(p.DIRECT)

    conn = None
    if tcp:
        TCP_IP = '127.0.0.1'
        TCP_PORT = 5005
        BUFFER_SIZE = 20  # Normally 1024, but we want fast response

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)

        conn, addr = s.accept()

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")

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

            high_level_controller.set_rate_target(ROLL, p.readUserDebugParameter(roll_rate_slider))

            lobster.set_buoyancy(p.readUserDebugParameter(buoyancyForceSlider))
            lobster.set_max_thrust(p.readUserDebugParameter(totalThrustSlider))

        velocity = p.getBaseVelocity(lobster.id)
        high_level_controller.update(lobster_pos, lobster_orn, velocity, desired_location)

        thrust_values = high_level_controller.motor_outputs

        for i in range(4):
            thrust_values[i] += forward_thrust

        lobster.set_thrust_values(thrust_values)
        lobster.update()

        if tcp:
            conn.send(bytes(str(lobster_pos), 'utf-8'))

        p.stepSimulation()
        # time.sleep(1. / 240.)

        if gui:
            move_camera_target(lobster.get_position())

    p.disconnect()


if __name__ == '__main__':
    main(sys.argv)
