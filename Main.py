import json

import pybullet as p

from lobster_simulator.tools.Plot import Plot
from control.HighLevelController import HighLevelController
from lobster_simulator.tools.Constants import *
from lobster_simulator.Simulator import Simulator


def move_camera_target(target):
    camera_info = p.getDebugVisualizerCamera()

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_info[10],
        cameraYaw=camera_info[8],
        cameraPitch=camera_info[9],
        cameraTargetPosition=target
    )


def read_config():
    with open('lobster_simulator/data/config.json', 'r') as f:
        config = json.load(f)

    return config


def main(gui=True, tcp=False):

    time_step = 4000

    simulator = Simulator(time_step, None, gui)

    # Only try to add debug sliders and visualisation when the gui is showing
    if gui:
        desired_pos_sliders = [
            p.addUserDebugParameter("desired x", -100, 100, 0),
            p.addUserDebugParameter("desired y", -100, 100, 0),
            p.addUserDebugParameter("desired z", -100, 0, -10)
        ]
        roll_rate_slider = p.addUserDebugParameter("rate ROLL", -10, 10, 0)
        debug_line = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=simulator.lobster.get_position(), lineWidth=5)

        simulator_time_step_slider = p.addUserDebugParameter("simulation timestep microseconds", 1000, 500000, 4000)

    high_level_controller = HighLevelController(gui)

    desired_location = [0, 0, 2]

    paused = False

    plot = Plot(3)

    while True:
        keys = p.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
            break
        if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
            # paused = not paused
            # if paused:
            plot.plot()

        if not paused:

            lobster_pos, lobster_orn = simulator.lobster.get_position_and_orientation()

            # Reading all the debug parameters (only if the gui is showing)
            if gui:
                time_step = p.readUserDebugParameter(simulator_time_step_slider)

                desired_location = [
                    p.readUserDebugParameter(desired_pos_sliders[0]),
                    p.readUserDebugParameter(desired_pos_sliders[1]),
                    p.readUserDebugParameter(desired_pos_sliders[2])
                ]
                # Add a line from the lobster to the origin
                p.addUserDebugLine(lineFromXYZ=desired_location, lineToXYZ=lobster_pos, replaceItemUniqueId=debug_line,
                                   lineWidth=5, lineColorRGB=[1, 0, 0])

                high_level_controller.set_target_rate(ROLL, p.readUserDebugParameter(roll_rate_slider))

            simulator.set_time_step(time_step)

            velocity = p.getBaseVelocity(simulator.lobster.id)
            high_level_controller.update(lobster_pos, lobster_orn, velocity, desired_location, time_step/1000000)

            rpm_motors = high_level_controller.motor_rpm_outputs

            simulator.set_rpm_motors(rpm_motors)

            simulator.do_step()

    p.disconnect()

