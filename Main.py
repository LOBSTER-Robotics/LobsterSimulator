import argparse
import json
import math
import time

from pkg_resources import resource_filename


from control.HighLevelController import HighLevelController
from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Terrain import Terrain
from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *
from lobster_simulator.Simulator import Simulator, Models
from lobster_simulator.tools.DebugVisualization import DebugLine
from lobster_simulator.tools.PybulletAPI import PybulletAPI

import pybullet as p


def read_config():
    with open('lobster_simulator/data/config.json', 'r') as f:
        config = json.load(f)

    return config


def main():

    parser = argparse.ArgumentParser("Lobster Simulator")
    parser.add_argument('--gui', type=bool, help='Run with or without GUI')
    args = parser.parse_args()

    gui = args.gui

    time_step = 4000

    simulator = Simulator(time_step, model=Models.SCOUT_ALPHA, config=None, gui=gui)

    # PybulletAPI.loadURDF(resource_filename("lobster_simulator", "data/terrain.urdf"), Vec3([0, 0, 100]))


    # Only try to add debug sliders and visualisation when the gui is showing
    if gui:
        desired_pos_sliders = [
            PybulletAPI.addUserDebugParameter("desired x", -100, 100, 0),
            PybulletAPI.addUserDebugParameter("desired y", -100, 100, 0),
            PybulletAPI.addUserDebugParameter("desired z", 0, 100, 90)
        ]
        roll_rate_slider = PybulletAPI.addUserDebugParameter("rate ROLL", -10, 10, 0)


        simulator_time_step_slider = PybulletAPI.addUserDebugParameter("simulation timestep microseconds", 1000, 500000, 4000)

    high_level_controller = HighLevelController(gui)

    desired_location = simulator.robot.get_position()
    desired_orientation = [0.0, 0.0, 0.0]

    terrain_loader = Terrain.perlin_noise_terrain(30)

    paused = False


    cycles = 0
    previous_time = time.time()
    cycles_per_second = 0
    
    while True:
        keys = PybulletAPI.getKeyboardEvents()
        if ord('p') in keys and keys[ord('p')] == PybulletAPI.KEY_WAS_TRIGGERED:
            paused = not paused

        lobster_pos, lobster_orn = simulator.robot.get_position_and_orientation()

        terrain_loader.update(lobster_pos)

        desired_location = Translation.vec3_rotate_vector_to_local(lobster_orn, desired_location)
        if ord('q') in keys and keys[ord('q')] == p.KEY_IS_DOWN:
            desired_location[Z] -= 0.004
        if ord('e') in keys and keys[ord('e')] == p.KEY_IS_DOWN:
            desired_location[Z] += 0.004
        if ord('w') in keys and keys[ord('w')] == p.KEY_IS_DOWN:
            desired_location[X] += 0.004
        if ord('s') in keys and keys[ord('s')] == p.KEY_IS_DOWN:
            desired_location[X] -= 0.004
        if ord('a') in keys and keys[ord('a')] == p.KEY_IS_DOWN:
            desired_location[Y] -= 0.004
        if ord('d') in keys and keys[ord('d')] == p.KEY_IS_DOWN:
            desired_location[Y] += 0.004
        desired_location = Translation.vec3_rotate_vector_to_world(lobster_orn, desired_location)

        if ord('j') in keys and keys[ord('j')] == p.KEY_IS_DOWN:
            desired_orientation[Z] -= 0.003
        if ord('l') in keys and keys[ord('l')] == p.KEY_IS_DOWN:
            desired_orientation[Z] += 0.003
        if ord('u') in keys and keys[ord('u')] == p.KEY_IS_DOWN:
            desired_orientation[X] -= 0.003
        if ord('o') in keys and keys[ord('o')] == p.KEY_IS_DOWN:
            desired_orientation[X] += 0.003
        if ord('i') in keys and keys[ord('i')] == p.KEY_IS_DOWN:
            desired_orientation[Y] -= 0.003
        if ord('k') in keys and keys[ord('k')] == p.KEY_IS_DOWN:
            desired_orientation[Y] += 0.003

        if not paused:

            # Reading all the debug parameters (only if the gui is showing)
            if gui:
                time_step = PybulletAPI.readUserDebugParameter(simulator_time_step_slider)

                # desired_location = [
                #     PybulletAPI.readUserDebugParameter(desired_pos_sliders[0]),
                #     PybulletAPI.readUserDebugParameter(desired_pos_sliders[1]),
                #     PybulletAPI.readUserDebugParameter(desired_pos_sliders[2])
                # ]

                high_level_controller.set_target_rate(ROLL, PybulletAPI.readUserDebugParameter(roll_rate_slider))

            simulator.set_time_step(time_step)

            velocity = PybulletAPI.getBaseVelocity(simulator.robot._id)
            high_level_controller.update(lobster_pos, lobster_orn, velocity[0], velocity[1],
                                         desired_location,
                                         PybulletAPI.getQuaternionFromEuler(Vec3(desired_orientation)), time_step/1000000)

            rpm_motors = high_level_controller.motor_rpm_outputs

            simulator.get_robot().set_desired_rpm_motors(rpm_motors)

            simulator.do_step()

            previous_weight = 0.99
            cycles_per_second = previous_weight * cycles_per_second + (1-previous_weight)/(time.time() - previous_time)

            # print(f'{cycles_per_second:.0f}', end='\r')
            previous_time = time.time()

    PybulletAPI.disconnect()


if __name__ == '__main__':
    main()

