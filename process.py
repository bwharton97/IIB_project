import cv2
import numpy as np
import matplotlib as plt
from mpl_toolkits.mplot3d import Axes3D

from constants import FRAMERATE, RESOLUTION
STAT_FREQ = 3  # In seconds


def process_and_display_stream(pisys):
    count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0
    while True:
        frame = pisys.get_synced_multiframe()

        # Network stats stuff
        dropped_frames += int(frame.was_frame_dropped)
        quick_reads += int(frame.are_all_quick_read())
        sum_latency += frame.get_avg_latency()
        count += 1
        if count > STAT_FREQ * FRAMERATE:
            # Print stats
            print("Average latency: {:.2f}s Dropped frames due to sync: {:.2f}% Quick read frames: {:.2f}%".format(
                sum_latency / count,
                dropped_frames / count * 100,
                quick_reads / count * 100))
            count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0

        # Process frame
        frame.process()

        # Show image
        cv2.imshow("Processed stream", frame.combine_frames())
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def process_recording(pisys):
    pisys.load_recording_from_disk()
    print("Processing and writing to disk...")
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    # Beware changing FRAMERATE or RESOLUTION without recording new video
    resolution = (2 * RESOLUTION[0], RESOLUTION[1])
    out = cv2.VideoWriter('processed_video.avi', fourcc, FRAMERATE, resolution)
    while True:
        frame = pisys.get_synced_multiframe()
        if frame is None:
            break
        frame.process()
        out.write(frame.combine_frames())
    out.release()


def play_processed_recording():
    print("Playing processed recording from disk...")
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

    #draw_graphs()