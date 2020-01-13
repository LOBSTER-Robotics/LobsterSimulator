import pybullet as p
import time
import pybullet_data

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

    lobster = LobsterScout(2, 0.2, 0.75, -0.3, -0.3)

    thrust_sliders = list()
    for i in range(6):
        thrust_sliders.append(p.addUserDebugParameter("motor" + str(i) + "Thrust", 0, 1, 0))

    debugLine = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), lineWidth=5)

    while True:

        p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=lobster.get_position(), replaceItemUniqueId=debugLine,
                           lineWidth=5, lineColorRGB=[1, 0, 0])

        thrust_values = [p.readUserDebugParameter(thrust_slider) for thrust_slider in thrust_sliders]
        lobster.update_motors(thrust_values)
        lobster.update()

        p.stepSimulation()
        time.sleep(1. / 240.)
        move_camera_target(lobster.get_position())

        # p.debug

    p.disconnect()


if __name__ == '__main__':
    main()