import noise
import numpy as np
import matplotlib.pyplot as plt


def perlin_array(shape,
                 offset,
                 distance_between_points,
                 scale=1000, octaves=6,
                 persistence=0.5,
                 lacunarity=2.0,
                 seed=1):
    arr = np.zeros(shape)
    for i in range(shape[0]):
        for j in range(shape[1]):
            world_x = (i + offset[0]) * distance_between_points
            world_y = (j + offset[1]) * distance_between_points

            print(world_x, world_y)

            arr[i][j] = noise.pnoise2(world_x / scale, world_y / scale,
                                      octaves=octaves,
                                      persistence=persistence,
                                      lacunarity=lacunarity,
                                      repeatx=1024,
                                      repeaty=1024,
                                      base=seed)

    return arr
#
#
# plt.imshow(np.concatenate([perlin_array(), perlin_array(offset=(200, 0))], 0), origin='upper')
# plt.show()
