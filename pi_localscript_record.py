import socket
import time
import picamera
import struct

RESOLUTION = (1920, 1080)
FRAMERATE = 30
DURATION = 60

server_socket = socket.socket()
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 8000))
server_socket.settimeout(10)
server_socket.listen(0)
print("Pi: Listening for connections")
connection = server_socket.accept()[0].makefile('wb')
print("Pi: Connection accepted")

try:
    with picamera.PiCamera(resolution=RESOLUTION, framerate=FRAMERATE) as camera:
        timestamp = time.time()
        connection.write(struct.pack('<d', timestamp))
        camera.start_recording(connection, format='h264')
        camera.wait_recording(DURATION)
        camera.stop_recording()
finally:
    connection.close()
    server_socket.close()
