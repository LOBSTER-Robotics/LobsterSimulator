import pybullet as p
import time
import pybullet_data
import numpy as np
import math


class LobsterScout:

    id = -1
    motorPositions = [
        [0.75, 0, -0.3],
        [0, 0.75, -0.3],
        [0, -0.75, -0.3],
        [-0.75, 0, -0.3]
    ]

    thrustSliders = list()
    thrusts = list()

    buoyancyForceSlider = None
    buoyancyXSlider = None
    buoyancyYSlider = None
    buoyancyZSlider = None

    totalThrustSlider = None

    buoyancySphereShape = None
    buoyancyPointIndicator = None


    def __init__(self):
        bodyId = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.2, height=2)
        headId = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)
        armId = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=1.5)

        linkOrientations = [
            [0, 0, 0, 1],
            p.getQuaternionFromEuler([math.pi / 2, 0, 0]),
            p.getQuaternionFromEuler([0, math.pi / 2, 0])
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

        linkVisualShapeIndices = [-1, -1, -1]

        linkPositions = [[0, 0, 1], [0, 0, -0.3], [0, 0, -0.3]]

        jointTypes = [p.JOINT_REVOLUTE,
                      p.JOINT_REVOLUTE,
                      p.JOINT_REVOLUTE]

        axis = [[0, 0, 1],
                [0, 0, 1],
                [0, 0, 1]]

        self.id = p.createMultiBody(
            baseMass                        = 10,
            baseCollisionShapeIndex         = bodyId,
            basePosition                    =[2, 2, 2],
            linkMasses                      =[2, 1, 1],
            linkVisualShapeIndices          = linkVisualShapeIndices,
            linkPositions                   = linkPositions,
            linkCollisionShapeIndices       = linkCollisionShapeIndices,
            linkOrientations                = linkOrientations,
            linkInertialFramePositions      = linkInertialFramePositions,
            linkInertialFrameOrientations   = linkInertialFrameOrientations,
            linkParentIndices               = indices,
            linkJointTypes                  = jointTypes,
            linkJointAxis                   = axis)

        p.changeDynamics(self.id, -1, linearDamping=0.9, angularDamping=0.9)
        for i in range(4):
            self.thrustSliders.append(p.addUserDebugParameter("motor" + str(i) + "Thrust", 0, 1000, 0))
            self.thrusts.append(p.readUserDebugParameter(self.thrustSliders[i]))

        self.buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 90)
        self.buoyancyXSlider = p.addUserDebugParameter("buoyancy x", -1, 1, 0)
        self.buoyancyYSlider = p.addUserDebugParameter("buoyancy y", -1, 1, 0)
        self.buoyancyZSlider = p.addUserDebugParameter("buoyancy z", -1, 1, 0)

        self.totalThrustSlider = p.addUserDebugParameter("total thrust", 0, 1, 0)

        self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.4])
        self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0], useMaximalCoordinates=0)

    def update(self):
        lobsterPos, lobsterOrn = p.getBasePositionAndOrientation(self.id)

        buoyancy = p.readUserDebugParameter(self.buoyancyForceSlider)
        buoyancyX = p.readUserDebugParameter(self.buoyancyXSlider)
        buoyancyY = p.readUserDebugParameter(self.buoyancyYSlider)
        buoyancyZ = p.readUserDebugParameter(self.buoyancyZSlider)

        totalThrust = p.readUserDebugParameter(self.totalThrustSlider)

        for i in range(4):
            self.thrusts[i] = p.readUserDebugParameter(self.thrustSliders[i])
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=[0, 0, self.thrusts[i] * totalThrust], posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)

        buoyancyForcePos = np.reshape(np.array(p.getMatrixFromQuaternion(lobsterOrn)), (3, 3)).dot(
            np.array([buoyancyX, buoyancyY, buoyancyZ])) \
                           + lobsterPos


        p.resetBasePositionAndOrientation(self.buoyancyPointIndicator, np.array(buoyancyForcePos), lobsterOrn)

        p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                             forceObj=[0, 0, buoyancy], posObj=np.array(buoyancyForcePos), flags=p.WORLD_FRAME)
