import pybullet as p
import time
import pybullet_data
import numpy as np
import math

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

colSphereId = p.createVisualShape(p.GEOM_SPHERE, radius=0.1)

bodyId = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.2, height=2)
headId = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)
armId = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=1.5)

linkOrientations = [
    [0, 0, 0, 1],
    p.getQuaternionFromEuler([math.pi/2, 0, 0]),
    p.getQuaternionFromEuler([0, math.pi/2, 0])
]
linkInertialFramePositions = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]]
linkVisualShapeIndices = [
    -1,
    -1,
    -1
]
linkInertialFrameOrientations = [
    [0, 0, 0, 1],
    [0, 0, 0, 1],
    [0, 0, 0, 1]
]

linkCollisionShapeIndices = [
    headId,
    armId,
    armId
]

indices = [
    0,
    0,
    0
]

linkVisualShapeIndices=[-1, -1, -1]

linkPositions = [[0, 0, 1], [0, 0, -0.3], [0, 0, -0.3]]

jointTypes = [p.JOINT_REVOLUTE,
              p.JOINT_REVOLUTE,
              p.JOINT_REVOLUTE]

axis = [[0, 0, 1],
        [0, 0, 1],
        [0, 0, 1]]

lobsterScoutId = p.createMultiBody(
    baseMass=10,
    baseCollisionShapeIndex=bodyId,
    basePosition=[2, 2, 2],
    linkMasses=[2, 1, 1],
    linkVisualShapeIndices=linkVisualShapeIndices,
    linkPositions=linkPositions,
    linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkOrientations=linkOrientations,
    linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations,
    linkParentIndices=indices,
    linkJointTypes=jointTypes,
    linkJointAxis=axis)

# p.changeDynamics(boxId, -1, linearDamping=0.9, angularDamping=0.9)

# p.setTimeStep(1./1440.)

buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 90)
buoyancyXSlider = p.addUserDebugParameter("buoyancy x", -1, 1, 0)
buoyancyYSlider = p.addUserDebugParameter("buoyancy y", -1, 1, 0)
buoyancyZSlider = p.addUserDebugParameter("buoyancy z", -1, 1, 0)

sphereUid = -1

sphereId = p.createMultiBody(0, -1, colSphereId, [0, 0, 0], useMaximalCoordinates=0)

secondsPassed=0
lastSecondsPrinted = 0
while(True):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(lobsterScoutId)

    buoyancy = p.readUserDebugParameter(buoyancyForceSlider)

    buoyancyX = p.readUserDebugParameter(buoyancyXSlider)
    buoyancyY = p.readUserDebugParameter(buoyancyYSlider)
    buoyancyZ = p.readUserDebugParameter(buoyancyZSlider)

    buoyancyForcePos = np.reshape(np.array(p.getMatrixFromQuaternion(cubeOrn)), (3, 3)).dot(
        np.array([buoyancyX, buoyancyY, buoyancyZ])) \
                       + cubePos

    # sphereUid = p.createMultiBody(0, -1, colSphereId, buoyancyForcePos, useMaximalCoordinates=0)
    p.resetBasePositionAndOrientation(sphereId, np.array(buoyancyForcePos), cubeOrn)

    p.applyExternalForce(objectUniqueId=lobsterScoutId, linkIndex=-1,
                         forceObj=[0, 0, buoyancy], posObj=np.array(buoyancyForcePos), flags=p.WORLD_FRAME)

    p.stepSimulation()
    # time.sleep(1. / 240.)
    secondsPassed += 1./240.
    if (secondsPassed - lastSecondsPrinted > 100):
        print(secondsPassed)
        lastSecondsPrinted = secondsPassed

p.disconnect()
