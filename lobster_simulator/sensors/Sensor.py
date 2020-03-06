

class Sensor:

    def __init__(self, pybullet_id, position, orientation, time_step):
        self.pybullet_id = pybullet_id
        self.position = position
        self.orientation = orientation
        self.time_step = time_step

    def update(self, dt):
        pass

    def get_sensor_position(self):
        return self.position

    def get_sensor_orientation(self):
        return self.orientation

