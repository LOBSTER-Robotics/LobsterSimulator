import json

import pybullet as p
import pybullet_data

from lobster_simulator.tools.Plot import Plot
from lobster_simulator.control.HighLevelController import HighLevelController
from lobster_simulator.robot.Lobster import Lobster
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.Logger import *
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
    with open('config.json', 'r') as f:
        config = json.load(f)

    return config


def main(gui=True, tcp=False):

    config = read_config()

    simulator = Simulator(1/240, config, gui)

    # Only try to add debug sliders and visualisation when the gui is showing
    if gui:
        forward_thrust_slider = p.addUserDebugParameter("forward thrust", -1, 1, 0)
        desired_pos_sliders = [
            p.addUserDebugParameter("desired x", -100, 100, 0),
            p.addUserDebugParameter("desired y", -100, 100, 0),
            p.addUserDebugParameter("desired z", 0, 100, 1)
        ]
        roll_rate_slider = p.addUserDebugParameter("rate ROLL", -10, 10, 0)
        buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 120)
        totalThrustSlider = p.addUserDebugParameter("Max thrust", 0, 1000, 100)
        debugLine = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=simulator.lobster.get_position(), lineWidth=5)

        simulator_frequency_slider = p.addUserDebugParameter("simulation frequency", 1, 1000, 240)

    high_level_controller = HighLevelController()

    desired_location = [0, 0, 2]

    logger.info("x, y, z, desired_x, desired_y, desired_z, pitch_rate, roll_rate, yaw_rate, target_pitch_rate, "
                "target_roll_rate, target_yaw_rate")

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

            # if cycle % p.readUserDebugParameter(simulator_frequency_slider) == 0:
            #     dt = (time.time() - previous_time)
            #     print(1 / dt, dt)
            #     previous_time = time.time()

            lobster_pos, lobster_orn = simulalobster.get_position_and_orientation()

            logger.store('pos', lobster_pos)
            plot.add('target roll rate', high_level_controller.target_rates[PITCH])
            plot.add('actual roll rate', high_level_controller.rates[PITCH])

            forward_thrust = 0

            # Reading all the debug parameters (only if the gui is showing)
            if gui:
                p.setTimeStep(1 / p.readUserDebugParameter(simulator_frequency_slider))

                desired_location = [
                    p.readUserDebugParameter(desired_pos_sliders[0]),
                    p.readUserDebugParameter(desired_pos_sliders[1]),
                    p.readUserDebugParameter(desired_pos_sliders[2])
                ]
                # Add a line from the lobster to the origin
                p.addUserDebugLine(lineFromXYZ=desired_location, lineToXYZ=lobster_pos, replaceItemUniqueId=debugLine,
                                   lineWidth=5, lineColorRGB=[1, 0, 0])
                forward_thrust = p.readUserDebugParameter(forward_thrust_slider)

                high_level_controller.set_target_rate(ROLL, p.readUserDebugParameter(roll_rate_slider))

                lobster.set_buoyancy(p.readUserDebugParameter(buoyancyForceSlider))
                lobster.set_max_thrust(p.readUserDebugParameter(totalThrustSlider))

            velocity = p.getBaseVelocity(lobster.id)
            high_level_controller.update(lobster_pos, lobster_orn, velocity, desired_location)

            thrust_values = high_level_controller.motor_outputs

            for i in range(4):
                thrust_values[i] += forward_thrust

            lobster.set_thrust_values(thrust_values)
            lobster.update()





    p.disconnect()

