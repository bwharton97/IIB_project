import video
from calibrate import calibrate_extrinsic
from pisetup import upload_and_run_picam, PI_IP_ADDRESSES, Pi


def main():
    pis = []
    for pi_id in range(len(PI_IP_ADDRESSES)):
        pi = Pi(pi_id, PI_IP_ADDRESSES[pi_id])
        pi.write_to_disk()
        pis.append(pi)

    try:
        #calibrate_extrinsic(pis)
        #video.process_video(pis)
        pass
    finally:
        print("Closing connections")
        for pi in pis:
            pi.close_connection()


if __name__ == '__main__':
    upload_and_run_picam()
    main()
