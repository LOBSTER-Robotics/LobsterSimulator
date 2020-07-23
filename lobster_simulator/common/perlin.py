import noise
import numpy as np
import matplotlib.pyplot as plt

def perlin_array(shape=(200, 200),
                 offset=(0, 0),
                 scale=1000, octaves=6,
                 persistence=0.5,
                 lacunarity=2.0,
                 seed=1):
    if not seed:
        seed = np.random.randint(0, 100)
        print("seed was {}".format(seed))

    arr = np.zeros(shape)
    for i in range(shape[0]):
        for j in range(shape[1]):
            x, y = (i + offset[0]) / scale, (j + offset[1]) / scale

            # print(x, y)

            arr[i][j] = noise.pnoise2(x, y,
                                octaves=octaves,
                                persistence=persistence,
                                lacunarity=lacunarity,
                                repeatx=1024,
                                repeaty=1024,
                                base=seed)
    # max_arr = np.max(arr)
    # min_arr = np.min(arr)
    # norm_me = lambda x: (x - min_arr) / (max_arr - min_arr)
    # norm_me = np.vectorize(norm_me)
    # arr = norm_me(arr)
    return arr
#
#
# plt.imshow(np.concatenate([perlin_array(), perlin_array(offset=(200, 0))], 0), origin='upper')
# plt.show()