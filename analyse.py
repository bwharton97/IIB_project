import matplotlib.pyplot as plt
import numpy as np
from sklearn.decomposition import PCA
from scipy.fft import fft, fftfreq
from scipy.signal import find_peaks_cwt
from mpl_toolkits.mplot3d import Axes3D


class Analyser:
    def __init__(self):
        self.data_3D = None
        self.data_reduced = None
        self.data_reduced_3D = None
        self.timestamps = None

        try:
            self.data_3D = np.load('processed_data/spot_locations.npy')
            self.timestamps = np.load('processed_data/timestamps.npy')
        except (FileNotFoundError, ValueError):
            print('WARNING: Cannot load analysis data. Please run processing again.')

    def set_data(self, spot_locations, timestamps):
        self.data_3D = np.array(spot_locations)
        self.timestamps = np.array(timestamps) - timestamps[0]  # Align timestamps to start at 0
        np.save('processed_data/spot_locations.npy', self.data_3D)
        np.save('processed_data/timestamps.npy', self.timestamps)

    def PCA_reduction(self):
        model = PCA(n_components=1)
        self.data_reduced = model.fit_transform(self.data_3D)
        self.data_reduced_3D = model.inverse_transform(self.data_reduced)

    def plot_3D(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x, y, z = self.data_3D[:, 0], self.data_3D[:, 2], -self.data_3D[:, 1]
        ax.plot(x, y, z, label='position')
        x2, y2, z2 = self.data_reduced_3D[:, 0], self.data_reduced_3D[:, 2], -self.data_reduced_3D[:, 1]
        ax.plot(x2, y2, z2, label='projected onto principal axis')
        ax.set_xlabel("x")
        ax.set_ylabel("y - distance from Pi0")
        ax.set_zlabel("z - height")
        max_range = np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()]).max() / 2.0
        mid_x = (x.max() + x.min()) * 0.5
        mid_y = (y.max() + y.min()) * 0.5
        mid_z = (z.max() + z.min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        plt.legend()
        plt.show()

    def plot_time(self):
        fig = plt.figure()
        ax = plt.axes()
        ax.plot(self.timestamps, self.data_reduced)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Displacement along principal axis (cm)")
        plt.show()

    def plot_fft(self):
        N = len(self.timestamps)
        T = (self.timestamps[-1] - self.timestamps[0]) / (N - 1)
        fig = plt.figure()
        ax = plt.axes()
        xf = fftfreq(N, T)[:N//2]
        yf = abs(2.0/N * fft(self.data_reduced.reshape((-1,)))[:N//2])
        ax.plot(xf, yf)

        peaks = find_peaks_cwt(yf, widths=np.arange(1, yf.shape[0]//5)) - 1
        print("Primary frequency: {:.4f}Hz".format(xf[peaks[0]]))
        plt.plot(xf[peaks], yf[peaks], "x")

        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Magnitude")
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        plt.show()