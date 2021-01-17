import socket
import struct
import subprocess as sp
import threading
import time
import glob
import cv2
import numpy as np

from constants import RESOLUTION, FRAMERATE, MODE

PI_IP_ADDRESSES = ['10.42.0.171', '10.42.0.239']
SERVER_IP = '10.42.0.1'

class Pi:
    def __init__(self, pi_id, ip_address):
        self.id = pi_id
        self.ip_address = ip_address
        self.connection = None
        self.capture = None
        self.current_pos = None
        self.start_timestamp = None

        if RESOLUTION[0] <= 1640:
            # Binning
            self.cam_mtx = np.array([[2714 / 2, 0, RESOLUTION[0] / 2],
                                     [0, 2714 / 2, RESOLUTION[1] / 2],
                                     [0, 0, 1]])
        else:
            self.cam_mtx = np.array([[2714, 0, RESOLUTION[0] / 2],
                                     [0, 2714, RESOLUTION[1] / 2],
                                     [0, 0, 1]])

        """
        try:
            self.cam_mtx = np.load(open('cam_mtx_' + str(self.id), 'rb'))
        except (FileNotFoundError, ValueError):
            self.cam_mtx = None
            print('WARNING: Cannot load camera matrix for Pi{}. Please recalibrate intrinsic parameters'.format(pi_id))
        """

        # Extrinsic camera parameters
        self.rvec = None
        self.tvec = None
        self.P = None

        # np.save(open('cam_mtx_' + str(self.id), 'wb'), self.cam_mtx)  # Might be useful later

    def check_time_sync(self):
        print('\nAttempting to determine Pi{} sync status:'.format(self.id))
        try:
            sp.run(['ssh', 'pi@' + self.ip_address, 'sudo', 'ntpq', '-p'], timeout=3, check=True)
        except:
            print('Failed to find Pi{} sync status'.format(self.id))

    def upload_localscripts(self):
        try:
            sp.run(['scp', 'pi_localscript.py', 'pi@' + self.ip_address + ':'], timeout=3, check=True)
            sp.run(['scp', 'constants.py', 'pi@' + self.ip_address + ':'], timeout=3, check=True)
            print('Successfully pushed localscripts to Pi{} at {}'.format(self.id, self.ip_address))
        except:
            print('WARNING: Pushing localscripts script to Pi{} at {} failed'.format(self.id, self.ip_address))

    def run_localscript(self):
        print('Commencing localscript on Pi{}'.format(self.id))
        sp.run(['ssh', 'pi@' + self.ip_address, 'python3', 'pi_localscript.py'], capture_output=True)

    def establish_connection(self):
        pi_socket = socket.socket()
        pi_socket.connect((self.ip_address, 8000))
        self.connection = pi_socket.makefile('rb')
        print('Successfully established connection with Pi{}'.format(self.id))

    def receive_and_write_to_disk(self):
        print('Receiving recording stream...')
        self.start_timestamp = struct.unpack('<d', self.connection.read(struct.calcsize('<d')))[0]
        recording_file = self.connection.read()
        filename = 'recordings/recording_pi{}_{}.h264'.format(self.id, self.start_timestamp)
        print('Recording received. Writing file {} to disk'.format(filename))
        output_file = open(filename, 'wb')
        output_file.write(recording_file)
        self.capture = cv2.VideoCapture(filename)
        self.current_pos = 0

    def record(self):
        """This method will hang until entire recording is finished."""
        # Start localscript on Pi. Threaded because runs simultaneously
        threading.Thread(target=self.run_localscript,
                         name='localscript-Pi' + str(self.id)).start()
        time.sleep(1.5)  # Allow time for Pi to create the socket connection

        self.establish_connection()
        self.receive_and_write_to_disk()  # This will hang until recording finished.
        self.connection.close()

    def load_recording_from_disk(self):
        filenames = glob.glob('recordings/recording_pi{}_*.h264'.format(self.id))
        filenames.sort(reverse=True)
        most_recent_filename = filenames[0]
        print('Loading most recent file', most_recent_filename, 'from disk')
        self.start_timestamp = float(most_recent_filename[25:-5])
        self.capture = cv2.VideoCapture(most_recent_filename)
        self.current_pos = 0
        print('Loading file complete for Pi{}'.format(self.id))

    def get_frame(self):
        if MODE == 'stream':
            time1 = time.time()
            frame_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
            timestamp = struct.unpack('<d', self.connection.read(struct.calcsize('<d')))[0]
            buffer = self.connection.read(frame_len)
            time2 = time.time()
            frame = cv2.imdecode(np.frombuffer(buffer, np.uint8), 1)  # This is operation that is problematically slow
            time3 = time.time()
            #print('Reading took {:.0f}% of frame period, decoding took {:.0f}%'.format((time2-time1)*FRAMERATE*100,
            #                                                                           (time3-time2)*FRAMERATE*100))
            quick_read = time2-time1 < 0.0003  # If read instant then buffer has data waiting, this is bad
        elif MODE == 'record':
            ret, frame = self.capture.read()
            timestamp = self.start_timestamp + self.current_pos
            self.current_pos += 1/FRAMERATE
            quick_read = False  # Not relevant for recording
        else:
            raise RuntimeError('Mode not recognised')

        frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate 180 deg while camera is upside down
        return frame, timestamp, quick_read


def record_from_pis(pis):
    # This must be outside the Pi class because it records from all Pis simultaneously
    recording_threads = []
    for pi in pis:
        thread = threading.Thread(target=pi.record, name='record-Pi' + str(pi.id))
        thread.start()
        recording_threads.append(thread)
    # Wait for recording threads to finish
    for thread in recording_threads:
        thread.join()


def start_stream(pis):
    # This must be outside the Pi class because it streams from all Pis simultaneously
    for pi in pis:
        threading.Thread(target=pi.run_localscript, name='localscript-Pi' + str(pi.id)).start()
    time.sleep(1.5)

    for pi in pis:
        pi.establish_connection()


if __name__ == '__main__':
    for pi_id in range(len(PI_IP_ADDRESSES)):
        pi = Pi(pi_id, MODE, PI_IP_ADDRESSES[pi_id])
        pi.check_time_sync()
