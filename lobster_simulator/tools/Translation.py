from typing import List

import numpy as np

#
# Functions that handle some of the conversions between local and world frame based vectors
#
from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.tools.PybulletAPI import PybulletAPI


def vec3_local_to_world(local_frame_position: Vec3, local_frame_orientation: Quaternion,
                        local_vec: Vec3) -> np.ndarray:
    """
    Converts a vector in a local reference frame to the global reference frame
    :param local_frame_position: Position of the local frame
    :param local_frame_orientation: Orientation of the local frame
    :param local_vec: Vector in the local reference frame
    :return: Vector in the global reference frame
    """
    return np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(local_frame_orientation)), (3, 3)).dot(
        np.array(local_vec.data)) \
           + local_frame_position


def vec3_world_to_local(local_frame_position: Vec3, local_frame_orientation: Quaternion,
                        world_vec: Vec3) -> np.ndarray:
    """
    Converts a vector in the global reference frame to a local reference frame
    :param local_frame_position: Position of the local frame
    :param local_frame_orientation: Orientation of the local frame
    :param world_vec: Vector in the global reference frame
    :return: Vector in the local reference frame
    """
    rotation_matrix = np.linalg.inv(
        np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(local_frame_orientation)), (3, 3)))

    return np.dot(
        rotation_matrix,
        (np.array(world_vec.data) - np.array(local_frame_position.data))
    )


def vec3_rotate_vector_to_local(local_frame_orientation: Quaternion, world_vec: Vec3) -> np.ndarray:
    rotation_matrix = np.linalg.inv(
        np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(local_frame_orientation)), (3, 3)))

    return np.dot(rotation_matrix, world_vec.data)


def vec3_local_to_world_id(local_frame_id: int, local_vec: Vec3) -> np.ndarray:
    pos, orn = PybulletAPI.getBasePositionAndOrientation(local_frame_id)

    print("local vec", local_vec)

    return vec3_local_to_world(pos, orn, local_vec)


def vec3_world_to_local_id(local_frame_id: int, world_vec: Vec3) -> np.ndarray:
    pos, orn = PybulletAPI.getBasePositionAndOrientation(local_frame_id)
    return vec3_world_to_local(pos, orn, world_vec)
