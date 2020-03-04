

class Sensor:

    def __init__(self, pybullet_id, position, orientation, frequency):
        self.pybullet_id = pybullet_id
        self.position = position
        self.orientation = orientation
        self.frequency = frequency

    def update(self, dt):
        pass

    def get_sensor_position(self):
        pass

    def get_sensor_orientation(self):
        pass

