import argparse
import json
import time

from control.HighLevelController import HighLevelController
from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.tools.Constants import *
from lobster_simulator.Simulator import Simulator, Models
from lobster_simulator.tools.DebugVisualization import DebugLine
from lobster_simulator.tools.PybulletAPI import PybulletAPI


def read_config():
    with open('lobster_simulator/data/config.json', 'r') as f:
        config = json.load(f)

    return config


def main():

    parser = argparse.ArgumentParser("Learning to See in the Dark PyTorch")
    parser.add_argument('--gui', type=bool, help='Run with or without GUI')
    args = parser.parse_args()

    gui = args.gui

    time_step = 4000

    simulator = Simulator(time_step, model=Models.SCOUT_ALPHA, config=None, gui=gui)

    # Only try to add debug sliders and visualisation when the gui is showing
    if gui:
        desired_pos_sliders = [
            PybulletAPI.addUserDebugParameter("desired x", -100, 100, 0),
            PybulletAPI.addUserDebugParameter("desired y", -100, 100, 10),
            PybulletAPI.addUserDebugParameter("desired z", 0, 100, 10)
        ]
        roll_rate_slider = PybulletAPI.addUserDebugParameter("rate ROLL", -10, 10, 0)
        debug_line = DebugLine(Vec3([0, 0, 0]), simulator.robot.get_position(), 5)

        simulator_time_step_slider = PybulletAPI.addUserDebugParameter("simulation timestep microseconds", 1000, 500000, 4000)

    high_level_controller = HighLevelController(gui)

    desired_location = [0, 0, 2]

    paused = False

    cycles = 0
    previous_time = time.time()
    cycles_per_second = 0

    while True:
        keys = PybulletAPI.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')] & PybulletAPI.KEY_WAS_TRIGGERED:
            break

        if not paused:

            lobster_pos, lobster_orn = simulator.robot.get_position_and_orientation()

            # Reading all the debug parameters (only if the gui is showing)
            if gui:
                time_step = PybulletAPI.readUserDebugParameter(simulator_time_step_slider)

                desired_location = [
                    PybulletAPI.readUserDebugParameter(desired_pos_sliders[0]),
                    PybulletAPI.readUserDebugParameter(desired_pos_sliders[1]),
                    PybulletAPI.readUserDebugParameter(desired_pos_sliders[2])
                ]
                # Add a line from the lobster to the origin
                debug_line.update(Vec3(desired_location), lobster_pos)

                high_level_controller.set_target_rate(ROLL, PybulletAPI.readUserDebugParameter(roll_rate_slider))

            simulator.set_time_step(time_step)

            velocity = PybulletAPI.getBaseVelocity(simulator.robot._id)
            high_level_controller.update(lobster_pos, lobster_orn, velocity[0], velocity[1], Vec3(desired_location), time_step/1000000)

            rpm_motors = high_level_controller.motor_rpm_outputs

            simulator.get_robot().set_desired_rpm_motors(rpm_motors)

            simulator.do_step()

            previous_weight = 0.99
            cycles_per_second = previous_weight * cycles_per_second + (1-previous_weight)/(time.time() - previous_time)

            print(f'{cycles_per_second:.0f}', end='\r')
            previous_time = time.time()

    PybulletAPI.disconnect()


if __name__ == '__main__':
    main()

