import math

from lobster_simulator.tools.PybulletAPI import PybulletAPI, Frame


class Motor:

    def __init__(self, pybullet_id, name, position, direction, rpm_to_thrust, thrust_to_rpm, max_rpm_change_per_second,
                 max_rpm):
        self._name = name
        self._pybullet_id = pybullet_id
        self._position = position
        self._direction = direction

        self._rpm_to_thrust = rpm_to_thrust
        self._thrust_to_rpm = thrust_to_rpm

        self._max_rpm_change_per_second = max_rpm_change_per_second
        self._max_rpm = max_rpm

        self._rpm = 0
        self._desired_rpm = 0

    @staticmethod
    def new_T200(pybullet_id, name, position, direction):
        rpm_to_thrust = lambda \
                rpm: 3.92 / 1000 * rpm + 3.9 / 10000000 * rpm * rpm + 7.55 / 10000000000 * rpm * rpm * rpm
        thrust_to_rpm = lambda x: -172.185 - 5.05393 * pow(261.54 * math.sqrt(
            3.84767 * math.pow(10, 8) * math.pow(x, 2) + 5.13478 * math.pow(10, 8) * x + 4.48941 * math.pow(10,
                                                                                                            9)) - 5.13023 * math.pow(
            10, 6) * x - 3.42319 * math.pow(10, 6), (1 / 3)) + 336577. / math.pow(261.54 * math.sqrt(
            3.84767 * pow(10, 8) * math.pow(x, 2) + 5.13478 * math.pow(10, 8) * x + 4.48941 * math.pow(10,
                                                                                                       9)) - 5.13023 * math.pow(
            10, 6) * x - 3.42319 * math.pow(10, 6), (1 / 3))

        return Motor(pybullet_id, name, position, direction, rpm_to_thrust, thrust_to_rpm,
                     max_rpm_change_per_second=40000, max_rpm=4000)

    def set_desired_rpm(self, desired_rpm):
        desired_rpm = min(desired_rpm, self._max_rpm)
        desired_rpm = max(desired_rpm, -self._max_rpm)

        self._desired_rpm = desired_rpm

    def set_desired_thrust(self, desired_thrust):
        self.set_desired_rpm(self._thrust_to_rpm(desired_thrust))

    def get_thrust(self):
        return self._rpm_to_thrust(self._rpm)

    def update(self, dt: int):
        """

        :param dt: dt in microseconds
        """
        diff = self._desired_rpm - self._rpm
        sign = int(diff > 0) - int(diff < 0)
        if math.fabs(diff) <= self._max_rpm_change_per_second * dt / 1000000:
            self._rpm = self._desired_rpm
        else:
            self._rpm += sign * self._max_rpm_change_per_second * dt / 1000000

    def apply_thrust(self):
        # p.applyExternalForce(objectUniqueId=self.pybullet_id, linkIndex=-1,
        #                      forceObj=self.direction * self.rpm_to_thrust(self.rpm),
        #                      posObj=self.position,
        #                      flags=p.LINK_FRAME)

        PybulletAPI.applyExternalForce(objectUniqueId=self._pybullet_id,
                                       forceObj=self._direction * self._rpm_to_thrust(self._rpm),
                                       posObj=self._position,
                                       frame=Frame.LINK_FRAME)
