"""A collection of functions for saving, recording and synchronising video"""
import time
import cv2

import process
from constants import MODE, FRAMERATE

STAT_FREQ = 3  # In seconds
if MODE == 'stream':
    SYNC_DIFFERENCE_LIMIT = 1 / FRAMERATE  # In seconds
else:
    SYNC_DIFFERENCE_LIMIT = 0.5 / FRAMERATE  # In seconds


def process_and_display_stream(pis):
    count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0
    while True:
        synced_frames, frame_drop, quick_read, latency = get_synced_frames(pis)

        # Network stats stuff
        dropped_frames += int(frame_drop)
        quick_reads += int(quick_read)
        sum_latency += latency
        count += 1
        if count > STAT_FREQ * FRAMERATE:
            # Print stats
            print("Average latency: {:.2f}s Dropped frames due to sync: {:.2f}% Quick read frames: {:.2f}%".format(
                sum_latency / count,
                dropped_frames / count * 100,
                quick_reads / count * 100))
            count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0

        # Process frame
        processed_frame = process.test_process_frames(pis, synced_frames)

        # Show image
        cv2.imshow("Processed stream", processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def get_synced_frames(pis):
    # Get next two frames which should be fairly in sync with each other.
    frames, timestamps, quick_reads = [], [], []
    for pi in pis:
        frame, timestamp, quick_read = pi.get_frame()
        frames.append(frame)
        timestamps.append(timestamp)
        quick_reads.append(quick_read)

    # Next part is currently only adapted to 2 Pis. If any frame is too far behind, then replace it with a new one
    frame_drop = False
    in_sync = False
    while not in_sync:
        if (timestamps[0] - timestamps[1]) > SYNC_DIFFERENCE_LIMIT:
            frames[1], timestamps[1], quick_reads[1] = pis[1].get_frame()
            frame_drop = True
            #print("Dropped a frame from pi1")
        elif (timestamps[1] - timestamps[0]) > SYNC_DIFFERENCE_LIMIT:
            frames[0], timestamps[0], quick_reads[0] = pis[0].get_frame()
            frame_drop = True
            #print("Dropped a frame from pi0")
        else:
            in_sync = True

    both_quick_read = (quick_reads[0] and quick_reads[1])
    avg_latency = time.time() - sum(timestamps) / len(timestamps)

    if frames[0] is None or frames[1] is None:
        # End of recorded video
        return None, frame_drop, both_quick_read, avg_latency
    return frames, frame_drop, both_quick_read, avg_latency


def process_recording(pis):
    for pi in pis:
        if pi.capture is None:
            pi.load_recording_from_disk()
    print("Processing from Pi objects and writing to disk...")
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    frames, frame_drop, both_quick_read, avg_latency = get_synced_frames(pis)
    # Beware changing FRAMERATE
    resolution = (2*frames[0].shape[1], frames[0].shape[0])
    out = cv2.VideoWriter('processed_video.avi', fourcc, FRAMERATE, resolution)  # Beware changing FRAMERATE
    while frames is not None:
        processed_frame = process.test_process_frames(pis, frames)
        out.write(processed_frame)
        frames, frame_drop, both_quick_read, avg_latency = get_synced_frames(pis)
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