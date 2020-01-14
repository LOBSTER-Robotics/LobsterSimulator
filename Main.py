import pybullet as p
import time
import pybullet_data
import math

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

    thrust_sliders = list()
    for i in range(6):
        thrust_sliders.append(p.addUserDebugParameter("motor" + str(i) + "Thrust", 0, 1, 0))

    debugLine = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), lineWidth=5)

    while True:

        p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), replaceItemUniqueId=debugLine,
                           lineWidth=5, lineColorRGB=[1, 0, 0])

        # thrust_values = [p.readUserDebugParameter(thrust_slider) for thrust_slider in thrust_sliders]

        angles = p.getEulerFromQuaternion(lobster.get_orientation())

        target_angles = [0, -math.pi/4, 0]
        delta_angles = [angles[0] - target_angles[0], angles[1] - target_angles[1], angles[2] - target_angles[2]]

        thrust_values = [0, 0, 0, 0, 0, 0]

        print(delta_angles[0] / math.pi, delta_angles[1] / math.pi, delta_angles[2] / math.pi)

        if delta_angles[0] > 0:
            thrust_values[3] = 10 * delta_angles[0] / math.pi
        else:
            thrust_values[2] = -10 * delta_angles[0] / math.pi
        #
        if delta_angles[1] > 0:
            thrust_values[0] = 10 * delta_angles[1] / math.pi
        else:
            thrust_values[1] = -10 * delta_angles[1] / math.pi

        if delta_angles[2] > 0:
            thrust_values[5] = 10 * delta_angles[2] / math.pi
        else:
            thrust_values[4] = -10 * delta_angles[2] / math.pi

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