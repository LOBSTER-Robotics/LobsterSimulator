import matplotlib.pyplot as plt


class Plot:

    def __init__(self, number_of_lines):
        self._x_data = list()
        self._y_data = dict()

    def add(self, line_name, y):
        if line_name not in self._y_data:
            self._y_data[line_name] = list()
        self._y_data[line_name].append(y)

    def plot(self):
        for value in self._y_data.values():
            plt.plot(value)

        plt.legend(self._y_data.keys())

        plt.show()
