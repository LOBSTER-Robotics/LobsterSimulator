import time

import numpy as np
import pybullet as p
import pybullet_data
import math

import Translation
from LobsterScout import LobsterScout
from RateController import RateController
from PID import PID


def move_camera_target(target):
    camera_info = p.getDebugVisualizerCamera()

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_info[10],
        cameraYaw=camera_info[8],
        cameraPitch=camera_info[9],
        cameraTargetPosition=target
    )


def main():
    PITCH = 0
    ROLL = 1
    YAW = 2

    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")

    lobster = LobsterScout(2, 0.2, 0.75, -0.3, 0)

    thrust_sliders = list()
    for i in range(6):
        thrust_sliders.append(p.addUserDebugParameter("motor" + str(i) + "Thrust", 0, 1, 0))

    rate_sliders = list()
    rate_sliders.append(p.addUserDebugParameter("rate PITCH", -10, 10, 0))
    rate_sliders.append(p.addUserDebugParameter("rate ROLL", -10, 10, 0))
    rate_sliders.append(p.addUserDebugParameter("rate YAW", -10, 10, 0))

    debugLine = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), lineWidth=5)

    rate_controller = RateController()

    orientation_pids = [
        PID(p=6, i=5, d=2, min_value=-10, max_value=10),
        PID(p=5, i=5, d=2, min_value=-10, max_value=10),
        PID(p=1, i=0, d=0, min_value=-10, max_value=10)
    ]

    desired_location = [0, 0, 0]

    while True:
        lobster_pos, lobster_orn = lobster.get_position_and_orientation()

        # Add a line from the lobster to the origin
        p.addUserDebugLine(lineFromXYZ=desired_location, lineToXYZ=lobster_pos, replaceItemUniqueId=debugLine,
                           lineWidth=5, lineColorRGB=[1, 0, 0])

        velocity = p.getBaseVelocity(lobster.id)

        # Translate world frame angular velocities to local frame angular velocities
        local_rotation = np.dot(
            np.linalg.inv(np.reshape(np.array(p.getMatrixFromQuaternion(lobster.get_orientation())), (3, 3))),
            velocity[1])

        relative_desired_location = Translation.vec3_world_to_local(
                lobster_pos,
                lobster_orn,
                [0, 0, 0]
            )

        relative_angles = [
            np.arctan2(relative_desired_location[1], relative_desired_location[2]),
            np.arctan2(relative_desired_location[0], relative_desired_location[2]),
            np.arctan2(relative_desired_location[1], relative_desired_location[0])
        ]

        relative_pitch = np.arctan2(relative_desired_location[1], relative_desired_location[2])
        relative_yaw = np.arctan2(relative_desired_location[0], relative_desired_location[2])

        print(end='\r')
        print(["{0:+0.2f}".format(i / math.pi) for i in relative_angles], end='')

        # print(end='\r')
        # print(relative_desired_location)

        # Desired rates
        target_rates = [p.readUserDebugParameter(rate_sliders[PITCH]),
                        p.readUserDebugParameter(rate_sliders[ROLL]),
                        p.readUserDebugParameter(rate_sliders[YAW])]

        orientation_pids[PITCH].update(relative_pitch, 1. / 240.)
        orientation_pids[YAW].update(-relative_yaw, 1. / 240.)

        target_rates[PITCH] = orientation_pids[PITCH].output
        target_rates[YAW] = orientation_pids[YAW].output

        rate_controller.set_desired_rates(target_rates)

        pitch_rate, yaw_rate, roll_rate = local_rotation

        rate_controller.update([pitch_rate, roll_rate, yaw_rate], 1. / 240.)

        # print(end='\r')
        # print("orn : (pitch: {0:+0.2f} roll: {1:+0.2f} yaw: {2:+0.2f})".format(local_orientation[0]/ math.pi, local_orientation[1], local_orientation[2]),
        #       end='')
        # print("rates : (pitch: {0:+0.2f} roll: {1:+0.2f} yaw: {2:+0.2f})".format(pitch_rate, roll_rate, yaw_rate), end='')
        #
        # print(["{0:+0.2f}".format(i) for i in rate_controller.rate_pids[YAW].get_terms()], end='')

        thrust_values = [0, 0, 0, 0, 0, 0]

        thrust_values[0] = -rate_controller.rate_pids[YAW].output
        thrust_values[1] =  rate_controller.rate_pids[YAW].output

        thrust_values[2] =  rate_controller.rate_pids[PITCH].output
        thrust_values[3] = -rate_controller.rate_pids[PITCH].output

        thrust_values[4] =  rate_controller.rate_pids[ROLL].output
        thrust_values[5] = -rate_controller.rate_pids[ROLL].output

        # print(["{0:+0.2f}".format(i) for i in thrust_values], end='')

        lobster.set_thrust_values(thrust_values)
        lobster.update()

        p.stepSimulation()
        time.sleep(1. / 240.)
        move_camera_target(lobster.get_position())

    p.disconnect()


if __name__ == '__main__':
    main()
