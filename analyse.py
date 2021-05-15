import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


class Analyser:
    def __init__(self):
        self.spot_locations = None
        self.timestamps = None

        try:
            self.spot_locations = np.load('processed_data/spot_locations.npy')
            self.timestamps = np.load('processed_data/timestamps.npy')
        except (FileNotFoundError, ValueError):
            print('WARNING: Cannot load analysis data. Please run processing again.')

    def set_data(self, spot_locations, timestamps):
        self.spot_locations = np.array(spot_locations)
        self.timestamps = timestamps
        np.save('processed_data/spot_locations.npy', self.spot_locations)
        np.save('processed_data/timestamps.npy', self.timestamps)

    def plot_3D(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x, y, z = self.spot_locations[:, 0], self.spot_locations[:, 2], -self.spot_locations[:, 1]
        ax.plot(x, y, z)
        ax.set_xlabel("x")
        ax.set_ylabel("y - distance from Pi0")
        ax.set_zlabel("z - height")
        plt.show()

    def plot_time(self):
        fig = plt.figure()
        ax = plt.axes()
        ax.plot(self.timestamps, self.spot_locations[:, 0])
        plt.show()