"""Contains the classes SingleViewFrame and MultiViewFrame, the latter can contain several of the former.
These also contain some methods which operate on the image"""
import cv2
import numpy as np
import time
from constants.constants import RESOLUTION


class SingleViewFrame:
    def __init__(self, frame, corresponding_pi, timestamp, quick_read):
        self.frame = frame
        self.corresponding_pi = corresponding_pi
        self.timestamp = timestamp
        self.quick_read = quick_read  # Used for stream mode

    def copy(self):
        return SingleViewFrame(self.frame.copy(), self.corresponding_pi, self.timestamp, self.quick_read)


class MultiViewFrame:
    """Contains multiple SingleViewFrames (currently 2 but could be expanded)"""
    def __init__(self, frames, was_frame_dropped):
        self.frames = frames
        self.timestamp = sum([frame.timestamp for frame in frames])/len(frames)
        self.was_frame_dropped = was_frame_dropped  # Used for for stream mode

        # Tracking characteristics
        self.spot_location_3D = None

    def copy(self):
        new_frames = []
        for frame in self.frames:
            new_frames.append(frame.copy())
        return MultiViewFrame(new_frames, self.was_frame_dropped)

    def are_all_quick_read(self):
        return all(frame.quick_read is True for frame in self.frames)

    def get_avg_latency(self):
        return time.time() - sum([frame.timestamp for frame in self.frames]) / len(self.frames)

    def combine_frames(self):
        """Returns concatenated image from multiple views for viewing"""
        return np.concatenate((self.frames[0].frame, self.frames[1].frame), axis=1)

    @staticmethod
    def locate_in_combined(x, y):
        """Takes a location in a combined image and returns the original coordinates and the id of the image it
        came from"""
        if x < RESOLUTION[0]:
            id = 0
            sub_x, sub_y = x, y
        else:
            id = 1
            sub_x, sub_y = x - RESOLUTION[0], y
        return sub_x, sub_y, id

    def add_marker(self, frame_id, coords, num, colour):
        """Draws a cross marker at the specified location in the image"""
        cv2.drawMarker(self.frames[frame_id].frame, coords, colour, cv2.MARKER_TILTED_CROSS, markerSize=50, thickness=2)
        cv2.putText(self.frames[frame_id].frame, str(num), (coords[0]+30, coords[1]),
                    cv2.FONT_ITALIC, 1, colour, 2)

    def draw_axis_lines(self, frame_id, img_points):
        img_points = img_points.astype(int)
        cv2.line(self.frames[frame_id].frame, tuple(img_points[0, 0]), tuple(img_points[1, 0]), (255, 0, 0), 2)
        cv2.line(self.frames[frame_id].frame, tuple(img_points[0, 0]), tuple(img_points[2, 0]), (0, 255, 0), 2)
        cv2.line(self.frames[frame_id].frame, tuple(img_points[0, 0]), tuple(img_points[3, 0]), (0, 0, 255), 2)

    def draw_axes(self, points3D):
        """Projects and draws basis vectors onto all views, using the 3D coordinates of the points and the calibration
        information"""
        for point in points3D:
            axis = point[0] + np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]).astype(float)

            for id in range(len(self.frames)):
                pi = self.frames[id].corresponding_pi
                img_points, jac = cv2.projectPoints(axis, pi.rvec, pi.tvec, pi.K, distCoeffs=None)
                self.draw_axis_lines(id, img_points)

    def triangulate_points(self, points):
        """Triangulate 3D location from points in two images. Would get more complicated if expanded to more views"""
        points4D = cv2.triangulatePoints(self.frames[0].corresponding_pi.P, self.frames[1].corresponding_pi.P,
                                         points[0].T, points[1].T).T
        points3D = cv2.convertPointsFromHomogeneous(points4D)
        return points3D

    def locate_spot(self):
        """Locates the brightest spot in the image and saves it to spot_location_3D"""
        SIGMA = 15
        spot_locations = []
        for i in range(len(self.frames)):
            frame = self.frames[i].frame
            grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred_frame = cv2.GaussianBlur(grey_frame, (0, 0), SIGMA)
            spot_location = np.unravel_index(np.argmax(blurred_frame, axis=None), blurred_frame.shape)[::-1]
            spot_locations.append(np.float32(spot_location))
            cv2.circle(frame, spot_location, SIGMA, (255, 0, 0), 2)
            self.frames[i].frame = frame

        spot_location_3D = self.triangulate_points(spot_locations)

        centre = np.array([[[0, 0, 0]]])
        #self.draw_axes(centre)
        if np.linalg.norm(spot_location_3D - centre) < 300:
            self.spot_location_3D = spot_location_3D
            self.draw_axes(self.spot_location_3D)
        else:
            self.spot_location_3D = None


