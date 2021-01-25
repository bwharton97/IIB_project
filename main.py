import video, calibrate
import pisetup


def main():
    # Create Pi objects
    pis = []
    for pi_id in range(len(pisetup.PI_IP_ADDRESSES)):
        pi = pisetup.Pi(pi_id, pisetup.PI_IP_ADDRESSES[pi_id])
        pis.append(pi)
        pi.upload_localscripts()  # Only necessary during development

    try:
        if pisetup.MODE == 'record':
            #pisetup.record_from_pis(pis)
            calibrate.calibrate_extrinsic_correspondences(pis)
            #video.process_recording(pis)
            #video.play_processed_recording()

        elif pisetup.MODE == 'stream':
            pisetup.start_stream(pis)
            calibrate.calibrate_extrinsic_chessboard(pis)
            video.process_and_display_stream(pis)

        else:
            raise RuntimeError("Mode not recognised")

    finally:
        print("Saving parameters and closing connections")
        for pi in pis:
            pi.save_extrinsic_params()
            if pi.connection is not None:
                pi.connection.close()


if __name__ == '__main__':
    main()
