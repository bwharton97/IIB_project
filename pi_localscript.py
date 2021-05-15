""" This script along with constants.py will be pushed and run remotely on the pi."""
import io
import sys
import socket
import struct
import time
import picamera

resolution = (int(sys.argv[1]), int(sys.argv[2]))
framerate = int(sys.argv[3])
mode = sys.argv[4]
duration = int(sys.argv[5])


class SplitFrames(object):
    def __init__(self, connection):
        self.connection = connection
        self.stream = io.BytesIO()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; send the old one's length then the old data
            frame_len = self.stream.tell()
            if frame_len > 0:
                timestamp = time.time()
                self.connection.write(struct.pack('<L', frame_len))
                self.connection.write(struct.pack('<d', timestamp))
                self.connection.flush()
                self.stream.seek(0)
                self.connection.write(self.stream.read(frame_len))
                """
                time2=time.time()
                writetime=time2-time1
                if writetime>1/framerate:
                    print("WARNING: delay in writing:", writetime)
                """
                self.stream.seek(0)
        self.stream.write(buf)


with picamera.PiCamera(resolution=resolution, framerate=framerate) as camera:
    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.settimeout(10)
    server_socket.listen(0)
    print("Pi: Listening for connections")
    connection = server_socket.accept()[0].makefile('wb')
    print("Pi: Connection accepted")

    if mode == 'stream':
        try:
            output = SplitFrames(connection)
            camera.start_recording(output, format='mjpeg')
            camera.wait_recording(24 * 60 * 60)
        except (BrokenPipeError, ConnectionResetError):
            # Interrupted somehow
            print("Pi: Interrupted")
            try:
                camera.stop_recording()
            except:
                pass
        server_socket.close()

    elif mode == 'record':
        try:
            timestamp = time.time()
            connection.write(struct.pack('<d', timestamp))
            camera.start_recording(connection, format='h264')
            camera.wait_recording(duration)
            camera.stop_recording()
        finally:
            connection.close()
            server_socket.close()

    else:
        raise RuntimeError("Mode not recognised")
