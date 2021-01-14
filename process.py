import cv2
import numpy as np


def process_frames(pis, frames):
    spot_locations = []
    for i in range(len(frames)):
        grey_frame = cv2.cvtColor(frames[i], cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(grey_frame,(101,101), 0)
        spot_location = np.unravel_index(np.argmax(blurred_frame, axis=None), blurred_frame.shape)
        spot_locations.append(np.float32(spot_location))

    points4D = cv2.triangulatePoints(pis[0].P, pis[1].P, np.flip(spot_locations[0]), np.flip(spot_locations[1]))
    points3D = cv2.convertPointsFromHomogeneous(points4D.T)[0,0]

    if np.linalg.norm(points3D)<200:
        # We have a fix
        print("3D position of light:", points3D)

        radius = int(1 * abs(points3D[2]))
        if points3D[2] > 0:
            colour = (0, 0, 255)
        else:
            colour = (0, 255, 0)
        for i in range(len(frames)):
            frames[i] = cv2.circle(frames[i], (spot_locations[i][1], spot_locations[i][0]), radius, colour, 3)

    processed_frame = np.concatenate((frames[0], frames[1]), axis=1)
    return processed_frame
