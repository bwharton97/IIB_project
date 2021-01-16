# This script will be pushed and run remotely on the pi.
import io
import socket
import struct
import time
import picamera

# RESOLUTION = (1640,922)
# RESOLUTION = (1920,1080)
RESOLUTION = (620, 480)
# RESOLUTION = (1640,922)
FRAMERATE = 8


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
                if writetime>1/FRAMERATE:
                    print("WARNING: delay in writing:", writetime)
                """
                self.stream.seek(0)
        self.stream.write(buf)


server_socket = socket.socket()
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 8000))
server_socket.settimeout(10)
server_socket.listen(0)
print("Pi: Listening for connections")
connection = server_socket.accept()[0].makefile('wb')
print("Pi: Connection accepted")
with picamera.PiCamera(resolution=RESOLUTION, framerate=FRAMERATE) as camera:
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
