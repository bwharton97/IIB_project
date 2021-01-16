import threading
import time
import sync
from calibrate import calibrate_extrinsic
import pisetup


def main():
    # Create Pi objects
    pis = []
    for pi_id in range(len(pisetup.PI_IP_ADDRESSES)):
        pi = pisetup.Pi(pi_id, pisetup.MODE, pisetup.PI_IP_ADDRESSES[pi_id])
        pis.append(pi)
        pi.upload_localscripts()  # Only necessary during development

    if pisetup.MODE == 'record':
        recording_threads = []
        for pi in pis:
            thread = threading.Thread(target=pi.record, name='record-Pi' + str(pi.id))
            thread.start()
            recording_threads.append(thread)

        # Wait for recording threads to finish
        for thread in recording_threads:
            thread.join()

    elif pisetup.MODE == 'stream':
        for pi in pis:
            threading.Thread(target=pi.run_localscript_stream, name='stream-Pi' + str(pi.id)).start()

        # Allow time for Pis to create the socket connection
        time.sleep(0.8)

        try:
            # calibrate_extrinsic(pis)
            # sync.process_video(pis)
            pass
        finally:
            print("Closing connections")
            for pi in pis:
                pi.close_connection()

    else:
        raise RuntimeError("Mode not recognised")


if __name__ == '__main__':
    main()
