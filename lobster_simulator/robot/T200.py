import math

from lobster_simulator.robot.Motor import Motor


class T200(Motor):

    def rpm_to_thrust(self, rpm):
        return 3.92 / 1000 * rpm + 3.9 / 10000000 * rpm * rpm + 7.55 / 10000000000 * rpm * rpm * rpm

    def __thrust_to_rpm(self, x):
        return -172.185 - 5.05393 * pow(261.54 * math.sqrt(
            3.84767 * math.pow(10, 8) * math.pow(x, 2) + 5.13478 * math.pow(10, 8) * x + 4.48941 * math.pow(10,
                                                                                                            9)) - 5.13023 * math.pow(
            10, 6) * x - 3.42319 * math.pow(10, 6), (1 / 3)) + 336577. / math.pow(261.54 * math.sqrt(
            3.84767 * pow(10, 8) * math.pow(x, 2) + 5.13478 * math.pow(10, 8) * x + 4.48941 * math.pow(10,
                                                                                                       9)) - 5.13023 * math.pow(
            10, 6) * x - 3.42319 * math.pow(10, 6), (1 / 3))
