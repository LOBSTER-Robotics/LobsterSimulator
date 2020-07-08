from typing import List

import numpy as np

#
# Functions that handle some of the conversions between local and world frame based vectors
#
from lobster_simulator.tools.PybulletAPI import PybulletAPI


def vec3_local_to_world(local_frame_position: List[float], local_frame_orientation: List[float], local_vec: List[float]) -> np.ndarray:
    """
    Converts a vector in a local reference frame to the global reference frame
    :param local_frame_position: Position of the local frame
    :param local_frame_orientation: Orientation of the local frame
    :param local_vec: Vector in the local reference frame
    :return: Vector in the global reference frame
    """
    return np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(local_frame_orientation)), (3, 3)).dot(
        np.array(local_vec)) \
                         + local_frame_position


def vec3_world_to_local(local_frame_position: List[float], local_frame_orientation: List[float], world_vec: List[float]) -> np.ndarray:
    """
    Converts a vector in the global reference frame to a local reference frame
    :param local_frame_position: Position of the local frame
    :param local_frame_orientation: Orientation of the local frame
    :param world_vec: Vector in the global reference frame
    :return: Vector in the local reference frame
    """
    rotation_matrix = np.linalg.inv(np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(local_frame_orientation)), (3, 3)))

    return np.dot(
        rotation_matrix,
        (np.array(world_vec) - np.array(local_frame_position))
    )


def vec3_rotate_vector_to_local(local_frame_orientation: List[float], world_vec: List[float]) -> np.ndarray:
    rotation_matrix = np.linalg.inv(np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(local_frame_orientation)), (3, 3)))

    return np.dot(rotation_matrix, np.array(world_vec))


def vec3_local_to_world_id(local_frame_id: int, local_vec: List[float]) -> np.ndarray:
    pos, orn = PybulletAPI.getBasePositionAndOrientation(local_frame_id)
    return vec3_local_to_world(pos, orn, local_vec)


def vec3_world_to_local_id(local_frame_id: int, world_vec: List[float]) -> np.ndarray:
    pos, orn = PybulletAPI.getBasePositionAndOrientation(local_frame_id)
    return vec3_world_to_local(pos, orn, world_vec)
