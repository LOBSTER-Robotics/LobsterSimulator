import matplotlib.pyplot as plt


class Plot:

    def __init__(self, number_of_lines):
        self.x_data = list()
        self.y_data = list()
        for i in range(number_of_lines):
            self.y_data.append(list())

    def add(self, x, y):
        self.x_data.append(x)
        for i in range(len(self.y_data)):
            self.y_data[i].append(y)

    def plot(self):
        print(self.y_data)
        for i in range(len(self.y_data)):
            plt.plot(self.x_data, self.y_data[i])

        plt.show()
