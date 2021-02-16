import cv2
import numpy as np

from constants import FRAMERATE, RESOLUTION
STAT_FREQ = 3  # In seconds


def process_frame(multiframe):
    """Accepts MultiViewFrame object and returns the same"""
    spot_locations = []
    for single_frame in multiframe.frames:
        grey_frame = cv2.cvtColor(single_frame.frame, cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(grey_frame, (101, 101), 0)
        spot_location = np.unravel_index(np.argmax(blurred_frame, axis=None), blurred_frame.shape)[::-1]
        spot_locations.append(np.float32(spot_location))
    points3D = multiframe.triangulate_points(spot_locations)
    #print(points3D)
    multiframe.draw_axes(points3D)

    #frame.draw_axes(np.array([[[0, 0, 500]]]))
    return multiframe


def old_process_frames(pisys, frames):
    """Not under development and not tested"""
    spot_locations = []
    for i in range(len(frames)):
        grey_frame = cv2.cvtColor(frames[i], cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(grey_frame, (101, 101), 0)
        spot_location = np.unravel_index(np.argmax(blurred_frame, axis=None), blurred_frame.shape)
        spot_locations.append(np.float32(spot_location))

    points4D = cv2.triangulatePoints(pisys.pis[0].P, pisys.pis[1].P,
                                     np.flip(spot_locations[0]), np.flip(spot_locations[1]))
    points3D = cv2.convertPointsFromHomogeneous(points4D.T)[0, 0]

    if np.linalg.norm(points3D)<200:
        # We have a fix
        print("3D position of light:", points3D)

        radius = int(1 * abs(points3D[2]))
        if points3D[2] > 0:
            colour = (0, 0, 255)
        else:
            colour = (0, 255, 0)
        for i in range(len(frames)):
            frames[i] = cv2.circle(frames[i], (spot_locations[i][1], spot_locations[i][0]), radius, colour, 3)

    processed_frame = np.concatenate((frames[0], frames[1]), axis=1)
    return processed_frame


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
        processed_frame = process_frame(frame)

        # Show image
        cv2.imshow("Processed stream", processed_frame.combine())
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
        processed_frame = process_frame(frame)
        out.write(processed_frame.combine())
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