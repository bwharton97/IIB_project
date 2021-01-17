import video
import pisetup


def main():
    # Create Pi objects
    pis = []
    for pi_id in range(len(pisetup.PI_IP_ADDRESSES)):
        pi = pisetup.Pi(pi_id, pisetup.PI_IP_ADDRESSES[pi_id])
        pis.append(pi)
        pi.upload_localscripts()  # Only necessary during development

    if pisetup.MODE == 'record':
        #pisetup.record_from_pis(pis)
        video.process_recording(pis)
        video.play_processed_recording()

    elif pisetup.MODE == 'stream':
        pisetup.start_stream(pis)

        try:
            # calibrate_extrinsic(pis)
            video.process_and_display_stream(pis)
            pass
        finally:
            print("Closing connections")
            for pi in pis:
                pi.connection.close()

    else:
        raise RuntimeError("Mode not recognised")


if __name__ == '__main__':
    main()
