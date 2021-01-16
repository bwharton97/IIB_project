import socket
import struct
import subprocess as sp
import threading
import time
import glob
import cv2
import numpy as np

PI_IP_ADDRESSES = ['10.42.0.171', '10.42.0.239']
SERVER_IP = '10.42.0.1'
MODE = 'record' #'stream'
FRAMERATE = 10  # Must match pi_localscript_stream.py, cannot just import this


class Pi:
    def __init__(self, pi_id, mode, ip_address):
        self.id = pi_id
        self.ip_address = ip_address
        self.mode = mode
        self.connection = None
        self.capture = None
        self.current_pos = None
        self.start_timestamp = None

        try:
            self.cam_mtx = np.load(open('cam_mtx_' + str(self.id), 'rb'))
        except (FileNotFoundError, ValueError):
            self.cam_mtx = None
            print("WARNING: Cannot load camera matrix for Pi{}. Please recalibrate intrinsic parameters".format(pi_id))
        #self.update_cam_mtx()

        # Extrinsic camera parameters
        self.rvec = None
        self.tvec = None
        self.P = None

    def check_time_sync(self):
        print("\nAttempting to determine Pi{} sync status:".format(self.id))
        try:
            sp.run(['ssh', 'pi@' + self.ip_address, 'sudo', 'ntpq', '-p'], timeout=3, check=True)
        except:
            print("Failed to find Pi{} sync status".format(self.id))

    def upload_localscripts(self):
        try:
            sp.run(['scp', 'pi_localscript_stream.py', 'pi@' + self.ip_address + ':'], timeout=3, check=True)
            sp.run(['scp', 'pi_localscript_record.py', 'pi@' + self.ip_address + ':'], timeout=3, check=True)
            print("Successfully pushed localscripts to Pi{} at {}".format(self.id, self.ip_address))
        except:
            print("WARNING: Pushing localscripts script to Pi{} at {} failed".format(self.id, self.ip_address))

    def run_localscript_stream(self):
        print("Commencing streaming on Pi{}".format(self.id))
        sp.run(['ssh', 'pi@' + self.ip_address, 'python3', 'pi_localscript_stream.py'], capture_output=True)

    def run_localscript_record(self):
        print("Commencing recording on Pi{}".format(self.id))
        sp.run(['ssh', 'pi@' + self.ip_address, 'python3', 'pi_localscript_record.py'], capture_output=False)

    def establish_connection(self):
        pi_socket = socket.socket()
        pi_socket.connect((self.ip_address, 8000))
        self.connection = pi_socket.makefile('rb')
        print("Successfully established connection with Pi{}".format(self.id))

    def read_and_write_to_disk(self):
        print("Receiving recording stream...")
        self.start_timestamp = struct.unpack('<d', self.connection.read(struct.calcsize('<d')))[0]
        recording_file = self.connection.read()
        print("Recording received. Writing file to disk")
        filename = "recordings/recording_pi{}_{}.h264".format(self.id, self.start_timestamp)
        output_file = open(filename, 'wb')
        output_file.write(recording_file)
        print("Writing file complete for Pi{}".format(self.id))
        self.capture = cv2.VideoCapture(filename)
        self.current_pos = 0

    def load_recording_from_disk(self):
        print("Loading file from disk")
        filenames = glob.glob('recordings/recording_pi{}_*.h264'.format(self.id))
        filenames.sort(reverse=True)
        most_recent_filename = filenames[0]
        self.start_timestamp = most_recent_filename[25:-5]
        self.capture = cv2.VideoCapture(most_recent_filename)
        self.current_pos = 0
        print("Loading file complete for Pi{}".format(self.id))

    def record(self):
        """This method will hang until entire recording is finished."""
        # Start recording on Pi. Threaded because runs simultaneously
        threading.Thread(target=self.run_localscript_record,
                         name='localscript_record-Pi' + str(self.id)).start()

        # Allow time for Pi to create the socket connection
        time.sleep(2)

        self.establish_connection()
        self.read_and_write_to_disk()  # This will hang until recording finished.

    def update_cam_mtx(self):
        """Takes one frame from camera to determine whether there is binning"""

        # Use first frame to determine camera matrix. Not the most efficient as this wastes a frame
        frame, timestamp, quick_read = self.get_frame()
        binning = frame.shape[1] <= 1640
        if not binning:
            self.cam_mtx = np.array([[2714, 0, frame.shape[1] / 2],
                                     [0, 2714, frame.shape[0] / 2],
                                     [0, 0, 1]])
        else:
            self.cam_mtx = np.array([[2714 / 2, 0, frame.shape[1] / 2],
                                     [0, 2714 / 2, frame.shape[0] / 2],
                                     [0, 0, 1]])

    def get_frame(self):
        if self.mode == 'stream':
            time1 = time.time()
            frame_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
            timestamp = struct.unpack('<d', self.connection.read(struct.calcsize('<d')))[0]
            buffer = self.connection.read(frame_len)
            time2 = time.time()
            quick_read = time2 - time1 < 0.0003  # If program reads instantly it means buffer has data waiting, this is bad
            frame = cv2.imdecode(np.frombuffer(buffer, np.uint8), 1)  # This is the operation that is problematically slow
            frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate 180 deg while camera is upside down
            return frame, timestamp, quick_read

        elif self.mode == 'record':
            ret, frame = self.capture.read()
            timestamp = self.start_timestamp + self.current_pos
            self.current_pos += 1/FRAMERATE
            return frame, timestamp, False

        else:
            raise RuntimeError("Mode not recognised")

    def close_connection(self):
        np.save(open('cam_mtx_' + str(self.id), 'wb'), self.cam_mtx)
        self.connection.close()
        self.socket.close()


if __name__ == '__main__':
    for pi_id in range(len(PI_IP_ADDRESSES)):
        pi = Pi(pi_id, MODE, PI_IP_ADDRESSES[pi_id])
        pi.load_recording_from_disk()
        pi.get_frame()