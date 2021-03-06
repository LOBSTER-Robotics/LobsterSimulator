import argparse
import json
import time

from control.HighLevelController import HighLevelController
from lobster_simulator.simulator import Simulator, Models
from lobster_common.vec3 import Vec3
from lobster_simulator.environment.terrain import Terrain
from lobster_common.constants import *
from lobster_simulator.common.pybullet_api import PybulletAPI


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

    simulator = Simulator(time_step, gui=gui,
                          config={"rotate_camera_with_robot": False})
    simulator.create_robot(Models.SCOUT_ALPHA)

    # Only try to add debug sliders and visualisation when the gui is showing
    if gui:
        desired_pos_sliders = [
            PybulletAPI.addUserDebugParameter("desired x", -100, 100, 0),
            PybulletAPI.addUserDebugParameter("desired y", -100, 100, 0),
            PybulletAPI.addUserDebugParameter("desired z", 0, 100, 90)
        ]
        roll_rate_slider = PybulletAPI.addUserDebugParameter("rate ROLL", -10, 10, 0)


        simulator_time_step_slider = PybulletAPI.addUserDebugParameter("simulation timestep microseconds", 1000, 500000, 4000)

    high_level_controller = HighLevelController(gui, simulator.robot.get_position(), Vec3([.0, .0, .0]), position_control=True)

    terrain_loader = Terrain.perlin_noise_terrain(30)

    paused = False

    cycles = 0
    previous_time = time.time()
    cycles_per_second = 0
    
    while True:
        keys = PybulletAPI.getKeyboardEvents()
        if ord('p') in keys and keys[ord('p')] == 3:
            paused = not paused
        if ord('r') in keys and keys[ord('r')] == 3:
            simulator.reset_robot()
        if PybulletAPI.DELETE_KEY in keys and keys[PybulletAPI.DELETE_KEY] == 3:
            break

        lobster_pos, lobster_orn = simulator.robot.get_position_and_orientation()

        terrain_loader.update(lobster_pos)

        if not paused:

            # Reading all the debug parameters (only if the gui is showing)
            if gui:
                time_step = PybulletAPI.readUserDebugParameter(simulator_time_step_slider)

                high_level_controller.set_target_rate(ROLL, PybulletAPI.readUserDebugParameter(roll_rate_slider))

            simulator.set_time_step(time_step)

            velocity = PybulletAPI.getBaseVelocity(simulator.robot.object_id)
            high_level_controller.update(lobster_pos, lobster_orn, velocity[0], velocity[1], time_step/1000000)

            thrust_motors = high_level_controller.motor_thrust_outputs

            for i, thruster in enumerate(simulator.robot.thrusters.values()):
                thruster.set_desired_thrust(thrust_motors[i])

            simulator.do_step()

            previous_weight = 0.99
            cycles_per_second = previous_weight * cycles_per_second + (1-previous_weight)/(time.time() - previous_time)

            # print(f'{cycles_per_second:.0f}', end='\r')
            previous_time = time.time()

    PybulletAPI.disconnect()


if __name__ == '__main__':
    main()

