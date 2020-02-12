import matplotlib.pyplot as plt


class Plot:

    def __init__(self, number_of_lines):
        self.x_data = list()
        self.y_data = dict()

    def add(self, line_name, y):
        if line_name not in self.y_data:
            self.y_data[line_name] = list()
        self.y_data[line_name].append(y)

    def plot(self):
        for value in self.y_data.values():
            plt.plot(value)

        plt.legend(self.y_data.keys())

        plt.show()
