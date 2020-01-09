import pybullet as p
import time
import pybullet_data
import numpy as np

from LobsterScout import LobsterScout


def main():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")

    lobster = LobsterScout()

    while (True):

        lobster.update()

        p.stepSimulation()
        time.sleep(1. / 240.)

    p.disconnect()


if __name__ == '__main__':
    main()