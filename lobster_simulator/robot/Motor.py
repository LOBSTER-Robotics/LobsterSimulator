import math

import pybullet as p


class Motor:

    def __init__(self, pybullet_id, name, position, direction, max_rpm_change_per_second):
        self.name = name
        self.pybullet_id = pybullet_id
        self.position = position
        self.direction = direction

        self.max_rpm_change_per_second = max_rpm_change_per_second

        self.rpm = 0
        self.desired_rpm = 0

    def rpm_to_thrust(self, rpm):
        # return 3.92 / 1000 * rpm + 3.9 / 10000000 * rpm * rpm + 7.55 / 10000000000 * rpm * rpm * rpm
        pass

    def __thrust_to_rpm(self, x):
        pass

    def set_desired_rpm(self, desired_rpm):
        self.desired_rpm = desired_rpm

    def set_desired_thrust(self, desired_thrust):
        self.desired_rpm = self.__thrust_to_rpm(desired_thrust)

    def get_thrust(self):
        return self.rpm_to_thrust(self.rpm)

    def update(self, dt):
        diff = self.desired_rpm - self.rpm
        sign = int(diff > 0) - int(diff < 0)
        if math.fabs(diff) <= self.max_rpm_change_per_second * dt:
            self.rpm = self.desired_rpm
        else:
            self.rpm += sign * self.max_rpm_change_per_second * dt

    def apply_thrust(self):
        p.applyExternalForce(objectUniqueId=self.pybullet_id, linkIndex=-1,
                             forceObj=self.direction * self.rpm_to_thrust(self.rpm),
                             posObj=self.position,
                             flags=p.LINK_FRAME)
