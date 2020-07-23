import numpy
import pybullet as p
import pybullet_data as pd
import math
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

textureId = -1

useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2
updateHeightfield = False

heightfieldSource = 0
import random

numHeightfieldRows = 2 ** 8
numHeightfieldColumns = 2 ** 8

def get_height(x, y):
    # math.sin(x / 100) * 10 +
    return math.sin(y / 10)

# def generate_height_map():

def load_chunk(x, y):
    # numHeightfieldRows = 2 ** 8
    # numHeightfieldColumns = 2 ** 8

    chunk_x = x * (numHeightfieldRows - 1)
    chunk_y = y * (numHeightfieldColumns - 1)

    heightfieldData = [0.0] * numHeightfieldRows * numHeightfieldColumns
    for j in range(numHeightfieldColumns):
        for i in range(numHeightfieldRows):

            world_x = (x * (numHeightfieldRows    - 1) + i) * 0.5
            world_y = (y * (numHeightfieldColumns - 1) + j) * 0.5

            height = get_height(world_x, world_y)
            heightfieldData[i + j*numHeightfieldRows] = height

    middle = (max(heightfieldData) + min(heightfieldData)) / 2
    # middle = heightfieldData[numHeightfieldRows + numHeightfieldColumns//2*numHeightfieldRows]

    terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.05, .05, 1],
                                          heightfieldTextureScaling=(numHeightfieldRows - 1),
                                          heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows,
                                          numHeightfieldColumns=numHeightfieldColumns)

    terrain = p.createMultiBody(0, terrainShape)
    p.resetBasePositionAndOrientation(terrain, [0.05 * (2**8-1) * x, 0.05 * (2**8-1) * y, middle], [0, 0, 0, 1])


random.seed(10)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
heightPerturbationRange = 0.05
if heightfieldSource == useProgrammatic:
    for i in range(-1, 2):
        for j in range(-1, 2):
            load_chunk(i, j)


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while (p.isConnected()):
    keys = p.getKeyboardEvents()

    if updateHeightfield and heightfieldSource == useProgrammatic:
        for j in range(int(numHeightfieldColumns / 2)):
            for i in range(int(numHeightfieldRows / 2)):
                height = random.uniform(0, heightPerturbationRange)  # +math.sin(time.time())
                heightfieldData[2 * i + 2 * j * numHeightfieldRows] = height
                heightfieldData[2 * i + 1 + 2 * j * numHeightfieldRows] = height
                heightfieldData[2 * i + (2 * j + 1) * numHeightfieldRows] = height
                heightfieldData[2 * i + 1 + (2 * j + 1) * numHeightfieldRows] = height
        # GEOM_CONCAVE_INTERNAL_EDGE may help avoid getting stuck at an internal (shared) edge of the triangle/heightfield.
        # GEOM_CONCAVE_INTERNAL_EDGE is a bit slower to build though.
        # flags = p.GEOM_CONCAVE_INTERNAL_EDGE
        flags = 0


        terrainShape2 = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, flags=flags, meshScale=[.05, .05, 1],
                                               heightfieldTextureScaling=(numHeightfieldRows - 1) / 2,
                                               heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows,
                                               numHeightfieldColumns=numHeightfieldColumns,
                                               replaceHeightfieldIndex=terrainShape)

    # print(keys)
    # getCameraImage note: software/TinyRenderer doesn't render/support heightfields!
    # p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    time.sleep(0.01)