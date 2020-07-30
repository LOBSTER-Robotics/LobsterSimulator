import threading

import inputs


class Gamepad:

    def __init__(self):

        print(list(inputs.devices))

        self.state = dict()
        self.running = False

        thread = threading.Thread(target=self._polling, args=())
        thread.start()  # Start the execution

    def _polling(self):
        self.running = True

        self._reset_state()

        while self.running:
            try:
                events = inputs.get_gamepad()
                for event in events:
                    self.state[event.code] = event.state
            except RuntimeError:
                self.state['ABS_X'] = 0
                self.state['ABS_Y'] = 0
                self.state['ABS_Z'] = 0
                self.state['ABS_RX'] = 0
                self.state['ABS_RY'] = 0
                self.state['ABS_RZ'] = 0
                break

        self._reset_state()

    def stop(self):
        self.running = False

    def _reset_state(self):
        self.state['ABS_X'] = 0
        self.state['ABS_Y'] = 0
        self.state['ABS_Z'] = 0
        self.state['ABS_RX'] = 0
        self.state['ABS_RY'] = 0
        self.state['ABS_RZ'] = 0

    @staticmethod
    def deadzone(input: float) -> float:
        sign = 1 if input >= 0 else -1
        if abs(input) < 0.1:
            return 0
        else:
            return (input - sign*0.1) * (10/9)

    @property
    def x(self):
        return self.deadzone(self.state['ABS_X'] / 32768)

    @property
    def y(self):
        return self.deadzone(self.state['ABS_Y'] / 32768)

    @property
    def z(self):
        return self.state['ABS_Z'] / 255

    @property
    def rx(self):
        return self.deadzone(self.state['ABS_RX'] / 32768)

    @property
    def ry(self):
        return self.deadzone(self.state['ABS_RY'] / 32768)

    @property
    def rz(self):
        return self.state['ABS_RZ'] / 255
