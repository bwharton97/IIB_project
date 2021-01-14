import time
import cv2

from process import process_frames

FRAMERATE = 10  # Must match pi_localscript_stream.py, cannot just import this
STAT_FREQ = 3  # In seconds
SYNC_DIFFERENCE_LIMIT = 1 / FRAMERATE  # In seconds


def process_video(pis):
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
            print("Average latency: {:.2f}s Dropped frames due to sync: {:.2f}%".format(sum_latency / count,
                                                                                        dropped_frames / count * 100))
            if quick_reads != 0:
                print("WARNING: Program is reading instantly from socket, suggesting program is running slower than "
                      "the network. Quick reads: {:.2f}%".format(quick_reads / count * 100))
            count, dropped_frames, quick_reads, sum_latency = 0, 0, 0, 0

        # Process frame
        processed_frame = process_frames(pis, synced_frames)

        # Show image
        cv2.imshow("frame", processed_frame)
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
            # print("Dropped a frame from pi1")
        elif (timestamps[1] - timestamps[0]) > SYNC_DIFFERENCE_LIMIT:
            frames[0], timestamps[0], quick_reads[0] = pis[0].get_frame()
            frame_drop = True
            # print("Dropped a frame from pi0")
        else:
            in_sync = True

    both_quick_read = (quick_reads[0] and quick_reads[1])
    avg_latency = time.time() - sum(timestamps) / len(timestamps)
    return frames, frame_drop, both_quick_read, avg_latency
