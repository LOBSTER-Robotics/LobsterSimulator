import matplotlib.pyplot as plt
import numpy as np


class AnimatedPlot:

    xData = []
    yData = []

    def __init__(self, identifier=''):
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13, 6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        self.line, = ax.plot(self.xData, self.yData, '-o', alpha=0.8)
        # update plot label/title
        plt.ylabel('Y Label')
        plt.title('Title: {}'.format(identifier))
        plt.show()

    def update(self, x, y):
        self.xData.append(x)
        self.yData.append(y)

        self.live_plotter()

    def live_plotter(self, pause_time=0.1):
        # after the figure, axis, and line are created, we only need to update the y-data
        self.line.set_ydata(self.yData)
        # adjust limits if new data goes beyond bounds
        # if np.min(self.y) <= self.line.axes.get_ylim()[0] or np.max(self.y) >= self.line.axes.get_ylim()[1]:
        #     plt.ylim([np.min(self.y) - np.std(self.y), np.max(self.y) + np.std(self.y)])
        # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
        plt.pause(pause_time)

