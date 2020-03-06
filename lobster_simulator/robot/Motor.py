import math

import pybullet as p


class Motor:

    def __init__(self, pybullet_id, name, position, direction, rpm_to_thrust, thrust_to_rpm, max_rpm_change_per_second):
        self.name = name
        self.pybullet_id = pybullet_id
        self.position = position
        self.direction = direction

        self.rpm_to_thrust = rpm_to_thrust
        self.thrust_to_rpm = thrust_to_rpm

        self.max_rpm_change_per_second = max_rpm_change_per_second

        self.rpm = 0
        self.desired_rpm = 0

    @staticmethod
    def new_T200(pybullet_id, name, position, direction):
        rpm_to_thrust = lambda rpm: 3.92 / 1000 * rpm + 3.9 / 10000000 * rpm * rpm + 7.55 / 10000000000 * rpm * rpm * rpm
        thrust_to_rpm = lambda x: -172.185 - 5.05393 * pow(261.54 * math.sqrt(3.84767 * math.pow(10, 8) * math.pow(x, 2) + 5.13478 * math.pow(10, 8) * x + 4.48941 * math.pow(10,9)) - 5.13023 * math.pow(10, 6) * x - 3.42319 * math.pow(10, 6), (1 / 3)) + 336577. / math.pow(261.54 * math.sqrt(3.84767 * pow(10, 8) * math.pow(x, 2) + 5.13478 * math.pow(10, 8) * x + 4.48941 * math.pow(10,9)) - 5.13023 * math.pow(10, 6) * x - 3.42319 * math.pow(10, 6), (1 / 3))

        return Motor(pybullet_id, name, position, direction, rpm_to_thrust, thrust_to_rpm, max_rpm_change_per_second=4000)

    def set_desired_rpm(self, desired_rpm):
        self.desired_rpm = desired_rpm

    def set_desired_thrust(self, desired_thrust):
        self.desired_rpm = self.__thrust_to_rpm(desired_thrust)

    def get_thrust(self):
        return self.rpm_to_thrust(self.rpm)

    def update(self, dt):
        diff = self.desired_rpm - self.rpm
        sign = int(diff > 0) - int(diff < 0)
        if math.fabs(diff) <= self.max_rpm_change_per_second * dt / 1000000:
            self.rpm = self.desired_rpm
        else:
            self.rpm += sign * self.max_rpm_change_per_second * dt / 1000000

    def apply_thrust(self):
        p.applyExternalForce(objectUniqueId=self.pybullet_id, linkIndex=-1,
                             forceObj=self.direction * self.rpm_to_thrust(self.rpm),
                             posObj=self.position,
                             flags=p.LINK_FRAME)
