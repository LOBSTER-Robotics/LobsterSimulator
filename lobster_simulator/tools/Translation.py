# This is needed to resolve the Lobster class type, since it can't be imported due to a cyclic dependency
from __future__ import annotations

#
# Functions that handle some of the conversions between local and world frame based vectors
#

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Vec3 import Vec3


def vec3_local_to_world(local_frame_position: Vec3, local_frame_orientation: Quaternion,
                        local_vec: Vec3) -> Vec3:
    """
    Converts a vector in a local reference frame to the global reference frame
    :param local_frame_position: Position of the local frame
    :param local_frame_orientation: Orientation of the local frame
    :param local_vec: Vector in the local reference frame
    :return: Vector in the global reference frame
    """
    word_vec = local_vec.rotate(local_frame_orientation) + local_frame_position
    return word_vec


def vec3_world_to_local(local_frame_position: Vec3, local_frame_orientation: Quaternion,
                        world_vec: Vec3) -> Vec3:
    """
    Converts a vector in the global reference frame to a local reference frame
    :param local_frame_position: Position of the local frame
    :param local_frame_orientation: Orientation of the local frame
    :param world_vec: Vector in the global reference frame
    :return: Vector in the local reference frame
    """
    return (world_vec - local_frame_position).rotate_inverse(local_frame_orientation)


def vec3_rotate_vector_to_world(local_frame_orientation: Quaternion, local_vec: Vec3) -> Vec3:
    return local_vec.rotate(local_frame_orientation)


def vec3_rotate_vector_to_local(local_frame_orientation: Quaternion, world_vec: Vec3) -> Vec3:
    return world_vec.rotate_inverse(local_frame_orientation)


def vec3_local_to_world_id(local_frame_id: int, local_vec: Vec3) -> Vec3:
    from lobster_simulator.tools.PybulletAPI import PybulletAPI
    pos, orn = PybulletAPI.getBasePositionAndOrientation(local_frame_id)
    return vec3_local_to_world(pos, orn, local_vec)


def vec3_world_to_local_id(local_frame_id: int, world_vec: Vec3) -> Vec3:
    from lobster_simulator.tools.PybulletAPI import PybulletAPI
    pos, orn = PybulletAPI.getBasePositionAndOrientation(local_frame_id)
    return vec3_world_to_local(pos, orn, world_vec)
