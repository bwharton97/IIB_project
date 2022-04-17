"""The class System contains the high-level functions and interfaces with the other classes"""
import cv2

from src.calibrate import Calibrator
from src.pisystem import PiSystem
from src.analyse import Analyser
from constants.constants import PI_IP_ADDRESSES, RESOLUTION, FRAMERATE, STAT_FREQ


class System:
    def __init__(self):
        self.pisys = PiSystem(PI_IP_ADDRESSES)
        self.analyser = Analyser()
        #self.pisys.upload_localscripts()  # Only necessary during development
        self.pisys.check_time_sync()  # Only necessary for diagnostics

    def record(self, duration):
        self.pisys.record(duration)

    def calibrate(self):
        Calibrator(self.pisys)

    def process_recording(self, max_frames_to_process=None):
        print("Processing and writing to disk...")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        # Beware changing FRAMERATE or RESOLUTION without recording new video
        resolution = (2 * RESOLUTION[0], RESOLUTION[1])
        out = cv2.VideoWriter('processed_video.avi', fourcc, FRAMERATE, resolution)

        spot_locations = []
        timestamps = []
        frame = self.pisys.get_synced_multiframe()
        n = 0
        while frame is not None and (max_frames_to_process is None or n < max_frames_to_process):
            n += 1
            frame.locate_spot()
            if frame.spot_location_3D is not None:
                spot_locations.append(frame.spot_location_3D[0, 0])
                timestamps.append(frame.timestamp)
            out.write(frame.combine_frames())
            frame = self.pisys.get_synced_multiframe()
        out.release()

        self.analyser.set_data(spot_locations, timestamps)

    def analyse(self):
        """Performs various analysis on the measured locations of the spot"""

        self.analyser.PCA_reduce()
        self.analyser.plot_3D()
        self.analyser.plot_time()
        self.analyser.plot_fft()

    def play_processed_recording(self):
        print("Playing processed recording from disk...")
        try:
            processed_capture = cv2.VideoCapture('processed_video.avi')
            frame_delay = round(1000 / FRAMERATE)
            while processed_capture.isOpened():
                ret, frame = processed_capture.read()
                if not ret:
                    print("End of video file")
                    break
                cv2.imshow('Processed video', frame)
                if cv2.waitKey(frame_delay) == ord('q'):
                    break
            processed_capture.release()
        except:
            print("Error opening or playing processed file")

    def stream(self):
        """Attempt to record, process and display the frame in real time"""
        try:
            self.pisys.start_stream()

            count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0
            while True:
                frame = self.pisys.get_synced_multiframe()

                # Network stats stuff
                dropped_frames += int(frame.was_frame_dropped)
                quick_reads += int(frame.are_all_quick_read())
                sum_latency += frame.get_avg_latency()
                count += 1
                if count > STAT_FREQ * FRAMERATE:
                    # Print stats
                    print(("Average latency: {:.2f}s Dropped frames due to sync:" +
                          "{:.2f}% Quick read frames: {:.2f}%").format(
                              sum_latency / count,
                              dropped_frames / count * 100,
                              quick_reads / count * 100))
                    count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0

                # Process frame
                #frame.locate_spot()

                # Show image
                cv2.imshow("Processed stream", frame.combine_frames())
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.pisys.close()
