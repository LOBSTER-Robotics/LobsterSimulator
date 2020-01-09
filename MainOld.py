import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)

colSphereId = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.4])

p.changeDynamics(boxId, -1, linearDamping=0.9, angularDamping=0.9)

# p.setTimeStep(1./1440.)

buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 650)
buoyancyXSlider = p.addUserDebugParameter("buoyancy x", -1, 1, 0)
buoyancyYSlider = p.addUserDebugParameter("buoyancy y", -1, 1, 0)
buoyancyZSlider = p.addUserDebugParameter("buoyancy z", -1, 1, 0)

sphereId = p.createMultiBody(0, -1, colSphereId, [0, 0, 0], useMaximalCoordinates=0)

while(True):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    spherePos, sphereOrn = p.getBasePositionAndOrientation(sphereId)

    buoyancy = p.readUserDebugParameter(buoyancyForceSlider)

    buoyancyX = p.readUserDebugParameter(buoyancyXSlider)
    buoyancyY = p.readUserDebugParameter(buoyancyYSlider)
    buoyancyZ = p.readUserDebugParameter(buoyancyZSlider)

    buoyancyForcePos = np.reshape(np.array(p.getMatrixFromQuaternion(cubeOrn)), (3, 3)).dot(np.array([buoyancyX, buoyancyY, buoyancyZ])) \
        + cubePos

    # sphereUid = p.createMultiBody(0, -1, colSphereId, buoyancyForcePos, useMaximalCoordinates=0)
    p.resetBasePositionAndOrientation(sphereId, np.array(buoyancyForcePos), cubeOrn)

    p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1,
                         forceObj=[0, 0, buoyancy], posObj=np.array(buoyancyForcePos), flags=p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()
