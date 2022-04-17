"""This module contains classes for each Pi and also the combined PiSystem, covering communication and camera fns."""
import socket
import struct
import subprocess as sp
import threading
import time
import glob
import cv2
import numpy as np

from constants.constants import RESOLUTION, FRAMERATE
from src.frame import SingleViewFrame, MultiViewFrame


class Pi:
    def __init__(self, id, ip_address):
        self.id = id
        self.ip_address = ip_address
        self.connection = None
        self.capture = None
        self.current_pos = None
        self.start_timestamp = None

        if RESOLUTION[0] <= 1640:
            # Binning
            self.K = np.array([[2714/2, 0, RESOLUTION[0]/2],
                               [0, 2714/2, RESOLUTION[1]/2],
                               [0, 0, 1]])
        else:
            self.K = np.array([[2714, 0, RESOLUTION[0]/2],
                               [0, 2714, RESOLUTION[1]/2],
                               [0, 0, 1]])

        # Extrinsic camera parameters rvec, tvec, P and E
        self.rvec, self.tvec, self.P, self.E = None, None, None, None
        try:
            # Check this is actually doing its job
            rvec = np.load('camera_params/rvec_{}.npy'.format(str(self.id)))
            tvec = np.load('camera_params/tvec_{}.npy'.format(str(self.id)))
            self.set_extrinsic_params(rvec, tvec)
            print('INFO: loaded saved camera parameters from disk')
        except (FileNotFoundError, ValueError):
            print('WARNING: Cannot load extrinsic parameters for Pi{}. Please recalibrate'.format(id))

    def save_extrinsic_params(self):
        np.save('camera_params/rvec_{}.npy'.format(str(self.id)), self.rvec)
        np.save('camera_params/tvec_{}.npy'.format(str(self.id)), self.tvec)

    def set_extrinsic_params(self, rvec, tvec):
        self.rvec = rvec
        self.tvec = tvec

        # Update derived params P and E
        R, jac = cv2.Rodrigues(self.rvec)
        Pr = np.concatenate((R, self.tvec), axis=1)
        self.P = np.matmul(self.K, Pr)
        # The following is the essential matrix compared to origin of coordinate system
        Tx = np.array([[0, -self.tvec[2, 0], self.tvec[1, 0]],
                       [self.tvec[2, 0], 0, -self.tvec[0, 0]],
                       [-self.tvec[1, 0], self.tvec[0, 0], 0]])
        self.E = np.matmul(Tx, R)

    def check_time_sync(self):
        print('\nPi{} sync status:'.format(self.id))
        try:
            sp.run(['ssh', 'pi@' + self.ip_address, 'sudo', 'ntpq', '-p'], timeout=3, check=True)
        except:
            print('Failed to find Pi{} sync status'.format(self.id))

    def upload_localscript(self):
        try:
            sp.run(['scp', 'pi_localscript.py', 'pi@' + self.ip_address + ':'], timeout=3, check=True)
            print('Successfully pushed localscript to Pi{} at {}'.format(self.id, self.ip_address))
        except:
            print('WARNING: Pushing localscript script to Pi{} at {} failed'.format(self.id, self.ip_address))

    def run_localscript(self, mode, duration):
        print('Commencing localscript on Pi{}'.format(self.id))
        sp.run(['ssh', 'pi@' + self.ip_address, 'python3', 'pi_localscript.py',
                str(RESOLUTION[0]), str(RESOLUTION[1]), str(FRAMERATE), mode, str(duration)], capture_output=True)

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

    def record(self, duration):
        """This method will hang until entire recording is finished."""
        # Start localscript on Pi. Threaded because runs simultaneously
        threading.Thread(target=self.run_localscript, name='localscript-Pi' + str(self.id),
                         args=('record', duration)).start()
        time.sleep(1.5)  # Allow time for Pi to create the socket connection

        self.establish_connection()
        self.receive_and_write_to_disk()  # This will hang until recording finished.
        self.connection.close()
        self.connection = None

    def load_recording_from_disk(self):
        filenames = glob.glob('recordings/recording_pi{}_*.h264'.format(self.id))
        filenames.sort(reverse=True)
        most_recent_filename = filenames[0]
        print('Loading most recent file', most_recent_filename, 'from disk')
        self.capture = cv2.VideoCapture(most_recent_filename)
        self.start_timestamp = float(most_recent_filename[25:-5])
        self.current_pos = 0

    def get_frame(self):
        if self.connection is not None:
            # Stream mode
            time1 = time.time()
            frame_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
            timestamp = struct.unpack('<d', self.connection.read(struct.calcsize('<d')))[0]
            buffer = self.connection.read(frame_len)
            time2 = time.time()
            frame = cv2.imdecode(np.frombuffer(buffer, np.uint8), 1)  # This is operation that is problematically slow
            #time3 = time.time()
            #print('Reading took {:.0f}% of frame period, decoding took {:.0f}%'.format((time2-time1)*FRAMERATE*100,
            #                                                                           (time3-time2)*FRAMERATE*100))
            quick_read = time2-time1 < 0.0003  # If read instant then buffer has data waiting, this is bad
        else:
            # Record mode
            if self.capture is None:
                self.load_recording_from_disk()
            ret, frame = self.capture.read()
            if not ret:  # End of video
                return None
            timestamp = self.start_timestamp + self.current_pos
            self.current_pos += 1/FRAMERATE
            quick_read = False  # Not relevant for recording

        frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate 180 deg while camera is upside down
        return SingleViewFrame(frame, self, timestamp, quick_read)


class PiSystem:
    def __init__(self, pi_ip_addresses):
        self.pis = []
        for id in range(len(pi_ip_addresses)):
            pi = Pi(id, pi_ip_addresses[id])
            self.pis.append(pi)
        self.mode = 'record'

    def upload_localscripts(self):
        for pi in self.pis:
            pi.upload_localscript()

    def record(self, duration):
        recording_threads = []
        for pi in self.pis:
            thread = threading.Thread(target=pi.record, name='record-Pi' + str(pi.id), args=(duration,))
            thread.start()
            recording_threads.append(thread)
        # Wait for recording threads to finish
        for thread in recording_threads:
            thread.join()

    def start_stream(self):
        self.mode = 'stream'
        for pi in self.pis:
            threading.Thread(target=pi.run_localscript, name='localscript-Pi' + str(pi.id),
                             args=('stream', 0)).start()
        time.sleep(1.5)

        for pi in self.pis:
            pi.establish_connection()

    def get_synced_multiframe(self):
        # Get next set of frames which should be fairly in sync with each other.

        if self.mode == 'record':
            sync_difference_limit = 0.5 / FRAMERATE  # In seconds
        else:
            sync_difference_limit = 1 / FRAMERATE  # In seconds

        frames = []
        for pi in self.pis:
            next_frame = pi.get_frame()
            frames.append(next_frame)

        # Next part is currently only adapted to 2 Pis. If any frame is too far behind, then replace it with a new one
        frame_drop = False
        in_sync = False
        while not in_sync:
            if None in frames:
                print("Warning: end of video reached.")
                return None
            elif (frames[0].timestamp - frames[1].timestamp) > sync_difference_limit:
                frames[1] = self.pis[1].get_frame()
                frame_drop = True
                #print("Dropped a frame from pi1")
            elif (frames[1].timestamp - frames[0].timestamp) > sync_difference_limit:
                frames[0] = self.pis[0].get_frame()
                frame_drop = True
                #print("Dropped a frame from pi0")
            else:
                in_sync = True

        return MultiViewFrame(frames, frame_drop)

    def check_time_sync(self):
        print("Checking Pi sync status. The host should be labelled with a * and the offset <10 (ms)")
        for pi in self.pis:
            pi.check_time_sync()

    def save_params(self):
        print("Saving external calibration parameters")
        for pi in self.pis:
            pi.save_extrinsic_params()

    def close(self):
        print("Closing connections")
        for pi in self.pis:
            if pi.connection is not None:
                pi.connection.close()