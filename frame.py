"""A collection of class and functions for frames and video"""
import cv2
import numpy as np
import time
from constants import RESOLUTION


class SingleViewFrame:
    def __init__(self, frame, corresponding_pi, timestamp, quick_read):
        self.frame = frame
        self.corresponding_pi = corresponding_pi
        self.timestamp = timestamp
        # Relevant to stream mode
        self.quick_read = quick_read

    def copy(self):
        return SingleViewFrame(self.frame.copy(), self.corresponding_pi, self.timestamp, self.quick_read)


class MultiViewFrame:
    def __init__(self, frames, was_frame_dropped):
        self.frames = frames
        # Properties for stream mode
        self.was_frame_dropped = was_frame_dropped

    def copy(self):
        new_frames = []
        for frame in self.frames:
            new_frames.append(frame.copy())
        return MultiViewFrame(new_frames, self.was_frame_dropped)

    def are_all_quick_read(self):
        return all(frame.quick_read is True for frame in self.frames)

    def get_avg_latency(self):
        return time.time() - sum([frame.timestamp for frame in self.frames]) / len(self.frames)

    def combine(self):
        """Returns concatenated image from multiple views"""
        return np.concatenate((self.frames[0].frame, self.frames[1].frame), axis=1)

    def locate_in_combined(self, x, y):
        if x < RESOLUTION[0]:
            id = 0
            sub_x, sub_y = x, y
        else:
            id = 1
            sub_x, sub_y = x - RESOLUTION[0], y
        return sub_x, sub_y, id

    def add_marker(self, frame_id, coords, colour):
        cv2.drawMarker(self.frames[frame_id].frame, coords, colour, cv2.MARKER_TILTED_CROSS, markerSize=50, thickness=2)

    def draw_axis_lines(self, frame_id, img_points):
        img_points = img_points.astype(int)
        cv2.line(self.frames[frame_id].frame, tuple(img_points[0, 0]), tuple(img_points[1, 0]), (255, 0, 0), 2)
        cv2.line(self.frames[frame_id].frame, tuple(img_points[0, 0]), tuple(img_points[2, 0]), (0, 255, 0), 2)
        cv2.line(self.frames[frame_id].frame, tuple(img_points[0, 0]), tuple(img_points[3, 0]), (0, 0, 255), 2)

    def draw_axes(self, points3D):
        for point in points3D:
            axis = point[0] + np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]).astype(float)

            for id in range(len(self.frames)):
                pi = self.frames[id].corresponding_pi
                img_points, jac = cv2.projectPoints(axis, pi.rvec, pi.tvec, pi.K, distCoeffs=None)
                self.draw_axis_lines(id, img_points)

    def triangulate_points(self, points):
        points4D = cv2.triangulatePoints(self.frames[0].corresponding_pi.P, self.frames[1].corresponding_pi.P,
                                         points[0].T, points[1].T).T
        points3D = cv2.convertPointsFromHomogeneous(points4D)
        return points3D
