import process
import calibrate
import pisystem

PI_IP_ADDRESSES = ['10.42.0.171', '10.42.0.239']
SERVER_IP = '10.42.0.1'


def main():
    # Create Pi objects
    pisys = pisystem.PiSystem(PI_IP_ADDRESSES)
    pisys.upload_localscripts()  # Only necessary during development
    #pisys.check_time_sync()

    try:
        if pisystem.MODE == 'record':
            #pisys.record_from_pis()
            calibrate.Calibrator(pisys)
            #process.process_recording(pisys)
            #process.play_processed_recording()

        elif pisystem.MODE == 'stream':
            pisys.start_stream()
            process.process_and_display_stream(pisys)

        else:
            raise RuntimeError("Mode not recognised")

    finally:
        pisys.close()


if __name__ == '__main__':
    main()
