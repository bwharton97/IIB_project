# Not sure whether this module is necessary.
import socket
import struct
import subprocess as sp
import threading
import time

import cv2
import numpy as np

PI_IP_ADDRESSES = ['10.42.0.171', '10.42.0.239']
SERVER_IP = '10.42.0.1'


def check_time_sync():
    # Check that Pi clocks are synchronised with laptop
    for pi_index in range(len(PI_IP_ADDRESSES)):
        print("\nAttempting to determine Pi{} sync status:".format(pi_index))
        try:
            sp.run(['ssh', 'pi@' + PI_IP_ADDRESSES[pi_index], 'sudo', 'ntpq', '-p'], timeout=3, check=True)
        except:
            print("Failed to find Pi{} sync status".format(pi_index))


def run_picam(pi_index):
    #sp.run(['ssh', 'pi@' + PI_IP_ADDRESSES[pi_index], 'python3', 'pi_localscript_stream.py'], capture_output=True)
    sp.run(['ssh', 'pi@' + PI_IP_ADDRESSES[pi_index], 'python3', 'pi_localscript_record.py'], capture_output=True)


def upload_picam():
    for pi_index in range(len(PI_IP_ADDRESSES)):
        try:
            #sp.run(['scp', 'pi_localscript_stream.py', 'pi@' + PI_IP_ADDRESSES[pi_index] + ':'], timeout=3, check=True)
            sp.run(['scp', 'pi_localscript_record.py', 'pi@' + PI_IP_ADDRESSES[pi_index] + ':'], timeout=3, check=True)
            print("Successfully pushed picam script to Pi{} at {}".format(pi_index, PI_IP_ADDRESSES[pi_index]))
        except:
            print("WARNING: Pushing picam script to Pi{} at {} failed".format(pi_index, PI_IP_ADDRESSES[pi_index]))


def upload_and_run_picam():
    upload_picam()
    for pi_index in range(len(PI_IP_ADDRESSES)):
        thread = threading.Thread(target=run_picam, args=[pi_index])
        thread.start()

    # Allow time for Pis to create the socket connection
    time.sleep(0.8)


class Pi:
    def __init__(self, pi_id, ip_address):
        # Establish connection
        self.id = pi_id
        self.ip_address = ip_address
        self.establish_connection()

        try:
            self.cam_mtx = np.load(open('cam_mtx_' + str(self.id), 'rb'))
        except (FileNotFoundError, ValueError):
            self.cam_mtx = None
            print("WARNING: Cannot load camera matrix for Pi{}. Please recalibrate intrinsic parameters".format(pi_id))
        #self.update_cam_mtx()

        # Extrinsic parameters
        self.rvec = None
        self.tvec = None
        self.P = None

    def establish_connection(self):
        self.socket = socket.socket()
        self.socket.connect((self.ip_address, 8000))
        self.connection = self.socket.makefile('rb')
        print("Successfully established connection with Pi{}".format(self.id))

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
        time1 = time.time()
        frame_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
        timestamp = struct.unpack('<d', self.connection.read(struct.calcsize('<d')))[0]
        buffer = self.connection.read(frame_len)
        time2 = time.time()
        quick_read = time2 - time1 < 0.0003  # If program reads instantly it means buffer has data waiting, this is bad
        frame = cv2.imdecode(np.frombuffer(buffer, np.uint8), 1)  # This is the operation that is problematically slow
        frame = cv2.rotate(frame, cv2.ROTATE_180) # Rotate 180 deg while camera is upside down
        return frame, timestamp, quick_read

    def write_to_disk(self):
        """Records entire output of stream to disk"""
        print("Writing file")
        output_file = open("recording_pi{}.h264".format(self.id), 'wb')
        output_file.write(self.connection.read())
        print("Writing file complete")

    def close_connection(self):
        np.save(open('cam_mtx_' + str(self.id), 'wb'), self.cam_mtx)
        self.connection.close()
        self.socket.close()


if __name__ == '__main__':
    check_time_sync()
