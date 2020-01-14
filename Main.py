import pybullet as p
import time
import pybullet_data
import math
import numpy as np

from LobsterScout import LobsterScout
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
    rate_sliders.append(p.addUserDebugParameter("rate PITCH", -1, 1, 0))
    rate_sliders.append(p.addUserDebugParameter("rate ROLL", -1, 1, 0))
    rate_sliders.append(p.addUserDebugParameter("rate YAW", -1, 1, 0))

    debugLine = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), lineWidth=5)

    rate_pids = [PID(p=1, i=0, d=0, min_value=-1, max_value=1),  # PITCH
                 PID(p=1, i=0, d=0, min_value=-1, max_value=1),  # ROLL
                 PID(p=1, i=0, d=0, min_value=-1, max_value=1)  # YAW
                 ]

    while True:

        p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), replaceItemUniqueId=debugLine,
                           lineWidth=5, lineColorRGB=[1, 0, 0])

        thrust_values = [p.readUserDebugParameter(thrust_slider) for thrust_slider in thrust_sliders]

        velocity = p.getBaseVelocity(lobster.id)

        # once we have that, we want to see what the world-frame rotations
        # are translated via rot_around_z
        local_rotation = np.dot(
            np.linalg.inv(np.reshape(np.array(p.getMatrixFromQuaternion(lobster.get_orientation())), (3, 3))),
            velocity[1])

        target_rates = [p.readUserDebugParameter(rate_sliders[PITCH]),
                        p.readUserDebugParameter(rate_sliders[ROLL]),
                        p.readUserDebugParameter(rate_sliders[YAW])]

        rate_pids[PITCH].set_target(target_rates[PITCH])
        rate_pids[ROLL].set_target(target_rates[ROLL])
        rate_pids[YAW].set_target(target_rates[YAW])

        delta_rates = [local_rotation[0] - target_rates[0],
                       local_rotation[1] - target_rates[1],
                       local_rotation[2] - target_rates[2]]

        rate_pids[PITCH].update(local_rotation[1], 1. / 240.)
        rate_pids[ROLL].update(local_rotation[0], 1. / 240.)
        rate_pids[YAW].update(local_rotation[1], 1. / 240.)

        print(end='\r')
        print("{0:+0.2f}".format(local_rotation[1]), end='')
        print(["{0:+0.2f}".format(i / math.pi) for i in local_rotation], end='')

        thrust_values = [0, 0, 0, 0, 0, 0]

        thrust_values[0] = - rate_pids[YAW].output
        thrust_values[1] =   rate_pids[YAW].output

        thrust_values[2] = rate_pids[PITCH].output
        thrust_values[3] = - rate_pids[PITCH].output
        #
        thrust_values[4] = rate_pids[ROLL].output
        thrust_values[5] = - rate_pids[ROLL].output



        #
        # print(end='\r')
        # print(["{0:+0.2f}".format(i / math.pi) for i in delta_angles], end='')
        # # print(delta_angles[0] / math.pi, delta_angles[1] / math.pi, delta_angles[2] / math.pi, end='')
        #
        # if delta_angles[0] > 0:
        #     thrust_values[3] = 10 * delta_angles[0] / math.pi
        # else:
        #     thrust_values[2] = -10 * delta_angles[0] / math.pi
        # #
        # if delta_angles[1] > 0:
        #     thrust_values[0] = 10 * delta_angles[1] / math.pi
        # else:
        #     thrust_values[1] = -10 * delta_angles[1] / math.pi
        #
        # if delta_angles[2] > 0:
        #     thrust_values[5] = 10 * delta_angles[2] / math.pi
        # else:
        #     thrust_values[4] = -10 * delta_angles[2] / math.pi

        # print(thrust_values)

        lobster.update_motors(thrust_values)
        lobster.update()

        p.stepSimulation()
        time.sleep(1. / 240.)
        # time.sleep(0.1)
        move_camera_target(lobster.get_position())

        # p.debug

    p.disconnect()


if __name__ == '__main__':
    main()
