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

colSphereId = p.createVisualShape(p.GEOM_SPHERE, radius=0.1)

p.changeDynamics(boxId, -1, linearDamping=0.9, angularDamping=0.9)

# p.setTimeStep(1./1440.)

buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 650)

sphereUid = -1

for i in range(100000):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

    buoyancy = p.readUserDebugParameter(buoyancyForceSlider)

    buoyancyForcePos = np.reshape(np.array(p.getMatrixFromQuaternion(cubeOrn)), (3, 3)).dot(np.array([0, 0, -0.5])) \
                       + cubePos

    sphereUid = p.createMultiBody(0, -1, colSphereId, buoyancyForcePos, useMaximalCoordinates=0)

    p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1,
                         forceObj=[0, 0, buoyancy], posObj=np.array(buoyancyForcePos), flags=p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()
