from src import system


def main():
    whole_system = system.System()
    whole_system.record(duration=20)
    whole_system.calibrate()
    whole_system.process_recording(max_frames_to_process=None)
    whole_system.analyse()
    whole_system.play_processed_recording()
    #whole_system.stream()


if __name__ == '__main__':
    main()
