import noise
import numpy as np
import matplotlib.pyplot as plt

def get_height_field_perlin2(chunk_x, chunk_y, points_per_chunk, point_spacing):
    height_field_data = [0.0] * points_per_chunk * points_per_chunk
    for j in range(points_per_chunk):
        for i in range(points_per_chunk):
            world_x = (-chunk_x * (points_per_chunk - 1) + i) * point_spacing
            world_y = (chunk_y * (points_per_chunk - 1) + j) * point_spacing

            scale = 80
            octaves = 6
            persistence = 0.5
            lacunarity = 2.0
            seed = 1

            print(world_x, world_y)

            height = noise.pnoise2(world_x / scale, world_y / scale,
                                   octaves=octaves,
                                   persistence=persistence,
                                   lacunarity=lacunarity,
                                   repeatx=1024,
                                   repeaty=1024,
                                   base=seed)
            height *= 200

            height_field_data[i + j * points_per_chunk] = height


    plt.show()

    return height_field_data

#
# points_per_chunk = 2 ** 7
# point_spacing = 2 ** 1
#
# get_height_field_perlin2(0, 0, points_per_chunk, point_spacing)
#
if __name__ == "__main__":
    points_per_chunk = 2 ** 7
    point_spacing = 2 ** 0

    height_field1 = get_height_field_perlin2(0, 0, points_per_chunk, point_spacing)
    height_field1 = np.reshape(height_field1, (-1, int(np.math.sqrt(len(height_field1)))))

    height_field2 = get_height_field_perlin2(-1, 0, points_per_chunk, point_spacing)
    height_field2 = np.reshape(height_field2, (-1, int(np.math.sqrt(len(height_field2)))))

    height_field3 = get_height_field_perlin2(0, 1, points_per_chunk, point_spacing)
    height_field3 = np.reshape(height_field3, (-1, int(np.math.sqrt(len(height_field3)))))

    height_field4 = get_height_field_perlin2(-1, 1, points_per_chunk, point_spacing)
    height_field4 = np.reshape(height_field4, (-1, int(np.math.sqrt(len(height_field4)))))

    height_field12 = np.concatenate([height_field1, height_field2], 1)
    height_field34 = np.concatenate([height_field3, height_field4], 1)

    plt.imshow(np.concatenate([height_field12, height_field34]), origin='upper')
    plt.show()

    points_per_chunk = 2 ** 7
    point_spacing = 2 ** 1

    height_field5 = get_height_field_perlin2(0, 0, points_per_chunk, point_spacing)
    height_field5 = np.reshape(height_field5, (-1, int(np.math.sqrt(len(height_field5)))))

    plt.imshow(height_field5, origin='upper')
    plt.show()
#
# points_per_chunk = 2 ** 7
# point_spacing = 2 ** -1
#
# get_height_field_perlin2(0, 0, points_per_chunk, point_spacing)