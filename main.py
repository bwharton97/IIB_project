import system


def main():
    sys = system.System()
    #sys.record(duration=10)
    #sys.calibrate()
    #sys.process_recording(max_frames_to_process=None)
    sys.analyse()
    #sys.play_processed_recording()
    #sys.stream()


if __name__ == '__main__':
    main()
