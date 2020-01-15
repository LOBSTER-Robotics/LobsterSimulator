from PID import PID
from Constants import *

class RateController:

    def __init__(self):
        self.desired_rates = [0, 0, 0]
        self.rate_pids = [PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1),  # PITCH
                          PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1),  # ROLL
                          PID(p=0.5, i=0.4, d=0.01, min_value=-1, max_value=1)  # YAW
                          ]

    def set_desired_rates(self, rates):
        for i in range(3):
            self.rate_pids[i].set_target(rates[i])

    def update(self, feedback_values, delta_time):
        for i in range(3):
            self.rate_pids[i].update(feedback_values[i], delta_time)