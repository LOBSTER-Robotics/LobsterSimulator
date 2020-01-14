
class PID:

    def __init__(self, p=0, i=0, d=0, min_value=0, max_value=1):
        self.kp = p
        self.ki = i
        self.kd = d

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.target = 0
        self.previous_error = None

        self.int_error = 0.0
        self.windup_guard = 20

        self.min = min_value
        self.max = max_value

        self.output = 0

    def update(self, feedback_value, delta_time):
        error = self.target - feedback_value
        delta_error = 0
        if self.previous_error:
            delta_error = error - self.previous_error

        # if delta_time >= self.sample_time:
        self.p_term = self.kp * error
        self.i_term += error * delta_time

        if self.i_term < -self.windup_guard:
            self.i_term = -self.windup_guard
        elif self.i_term > self.windup_guard:
            self.i_term = self.windup_guard

        self.d_term = 0.0
        if delta_time > 0:
            self.d_term = delta_error / delta_time

        self.previous_error = error

        self.output = self.p_term + (self.ki * self.i_term) + (self.kd * self.d_term)

        # if self.output > self.max:
        #     self.output = self.max
        # elif self.output < self.min:
        #     self.output = self.min

    def set_target(self, target):
        self.target = target
