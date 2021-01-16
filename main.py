import sync
from calibrate import calibrate_extrinsic
import pisetup


def main():
    pisetup.upload_localscripts()
    if pisetup.MODE == 'record':
        pisetup.record()

    elif pisetup.MODE == 'stream':
        pis = []
        for pi_id in range(len(pisetup.PI_IP_ADDRESSES)):
            pi = pisetup.Pi(pi_id, pisetup.PI_IP_ADDRESSES[pi_id])
            pis.append(pi)

        try:
            # calibrate_extrinsic(pis)
            # sync.process_video(pis)
            pass
        finally:
            print("Closing connections")
            for pi in pis:
                pi.close_connection()

    else:
        print("Mode not recognised, should be either stream or record.")


if __name__ == '__main__':
    main()
