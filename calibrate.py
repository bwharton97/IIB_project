import cv2
import numpy as np
import time
import frame


class Calibrator():
    def __init__(self, pisys):
        self.pisys = pisys
        self.original_frame = pisys.get_synced_multiframe()
        self.drawing_frame = self.original_frame.copy()
        self.correspondences, self.order_added = [[], []], []  # correpondences[0] is a list of lists for Pi0
        self.correspondences_3D = None
        self.colours = np.random.rand(100, 3) * 255  # 100 is arbitrary
        self.method = cv2.FM_8POINT  # Default method
        # real_distance = float(input("Enter distance in cm between the first two points you select:"))
        self.distance_first_2 = 60.5
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        self.selection_loop()

    def redraw(self):
        self.drawing_frame = self.original_frame.copy()
        self.draw_correspondence_markers()
        ret = self.calibrate()
        if ret:
            self.drawing_frame.draw_axes(self.correspondences_3D)
            if self.method == cv2.FM_8POINT:
                method = "Method: 8-point"
            else:
                method = "Method: LMedS"
            cv2.putText(self.drawing_frame.frames[0].frame, method, (30, 30), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)

    def draw_correspondence_markers(self):
        """Adds markers for each of these correspondences to drawing_frame"""
        for id in range(len(self.correspondences)):
            for n in range(len(self.correspondences[id])):
                self.drawing_frame.add_marker(id, self.correspondences[id][n], n, self.colours[n])

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            sub_x, sub_y, id = self.drawing_frame.locate_in_combined(x, y)

            n = len(self.correspondences[id])
            if n == len(min(self.correspondences, key=len)):
                self.correspondences[id].append((sub_x, sub_y))
                self.order_added.append(id)
                self.redraw()
            else:
                print("Please select matching correspondence on other image")

    def calibrate(self):
        """Calibrates Pi objects using correspondences (must be rectangular dims), prints info messages.
        Finds correspondences_3D"""
        # Trim correspondences
        correspondences = [view[:len(min(self.correspondences, key=len))] for view in self.correspondences]
        if len(correspondences[0]) < 7:
            # Too few points
            self.correspondences_3D = None
            return False
        print("\n----------New calibration:-----------")
        #print("Correspondences:\n", correspondences)
        # Calculate external params from correspondences
        # This part assumes both cameras have the same camera matrix. Use cv2.undistortPoints to improve
        points = np.array(correspondences).astype(float)
        F, F_mask = cv2.findFundamentalMat(points[0], points[1], method=self.method)

        # Evaluating the effectiveness of the F matrix
        for i in range(len(F_mask)):
            if F_mask[i] == 0:
                print("Warning: point {} rejected as outlier.".format(i))
        points_homo = cv2.convertPointsToHomogeneous(points.reshape((-1, 2))).reshape((2, len(points[0]), 3))
        try:
            error = np.diagonal(np.matmul(np.matmul(points_homo[1], F), points_homo[0].T)).reshape((-1, 1))
            print("Error in F:")
            for i in range(len(error)):
                if F_mask[i] == 0:
                    print("Point {}: {} (outlier)".format(i, error[i]))
                else:
                    print("Point {}: {}".format(i, error[i]))
        except ValueError as message:
            print(message)
            print(points_homo)
            print(F)

        # Recover pose and calibrate pis
        K = self.pisys.pis[0].K
        E = np.matmul(K.T, np.matmul(F, K))
        retval, R, tvec, pose_mask = cv2.recoverPose(E, points[0], points[1], K)
        rvec, jac = cv2.Rodrigues(R)
        self.pisys.pis[0].set_extrinsic_params(np.array([[0.0], [0.0], [0.0]]), np.array([[0.0], [0.0], [0.0]]))
        self.pisys.pis[1].set_extrinsic_params(rvec, tvec)

        # Calculate 3D position of correspondences and calibrate for scale using fist two points
        self.correspondences_3D = self.drawing_frame.triangulate_points(points)
        old_distance_first_2 = np.linalg.norm(self.correspondences_3D[0, 0] - self.correspondences_3D[1, 0])
        factor = self.distance_first_2 / old_distance_first_2
        self.correspondences_3D *= factor
        for pi in self.pisys.pis:
            new_tvec = pi.tvec * factor
            pi.set_extrinsic_params(pi.rvec, new_tvec)

        # Output info messages
        print("Correspondences in 3D:\n", self.correspondences_3D.reshape((-1, 3)))
        print("Vector between cameras:", self.pisys.pis[1].tvec.flatten())
        print("Distance between cameras:", np.linalg.norm(self.pisys.pis[1].tvec))
        return True

    def selection_loop(self):
        print("Please select correspondences.\n",
              "Controls: a = abort, f = finish, u = undo, n = next frame,\n",
              "          c = clear frame, d = delete point, m = toggle method")

        cv2.namedWindow('Select correspondences')
        cv2.setMouseCallback('Select correspondences', self.click_event)
        while True:  # len(min(self.correspondences, key=len)) < self.num_correspondences:
            cv2.imshow('Select correspondences', self.drawing_frame.combine_frames())
            key = cv2.waitKey(20) & 0xFF
            if key == ord('a'):
                # Abort
                return
            elif key == ord('f'):
                # Finish
                self.pisys.save_params()
                return
            elif key == ord('n'):
                # New frame
                self.original_frame = self.pisys.get_synced_multiframe()
                self.drawing_frame = self.original_frame.copy()
                self.correspondences, self.order_added = [[], []], []
            elif key == ord('c'):
                print("Clear correspondences")
                self.drawing_frame = self.original_frame.copy()
                self.correspondences, self.order_added = [[], []], []
            elif key == ord('u'):
                print("Undo last correspondence")
                try:
                    self.correspondences[self.order_added[-1]].pop()
                    self.order_added.pop()
                    self.redraw()
                except IndexError:
                    pass
            elif key == ord('d'):
                n = 'blank'
                while not n.isnumeric():
                    n = input("Enter index of point to delete:")
                n = int(n)
                found = False
                for i in range(len(self.correspondences)):
                    try:
                        self.correspondences[i].pop(n)
                        self.order_added = []  # Gonna have to just forget
                        found = True
                    except IndexError:
                        pass
                if found:
                    self.redraw()
                else:
                    print("Point not found. Out of range?")
            elif key == ord("m"):
                print("Toggle method")
                if self.method == cv2.FM_8POINT:
                    self.method = cv2.LMEDS
                else:
                    self.method = cv2.FM_8POINT
                self.redraw()


CHESSBOARD_SIZE = 2.42  # real-life size of a square in cm


def calibrate_extrinsic_chessboard(pis):
    """Function no longer in development"""
    all_found = False
    while not all_found:
        display_frames = []

        # Get and show new frames
        frames, frame_drop, both_quick_read, avg_latency = frame.get_synced_frames(pis)

        # Search for chessboards and attempt to locate them
        all_found = True
        for pi_index in range(len(pis)):
            ret, objpoints, imgpoints, frame_with_corners = detect_chessboard(frames[pi_index])
            if ret:
                display_frames.append(frame_with_corners)
                ret, rvec, tvec, inliers = cv2.solvePnPRansac(objpoints, imgpoints,
                                                              pis[pi_index].K, distCoeffs=None)
                if ret:
                    # Calculate projection matrices too
                    pis[pi_index].rvec = rvec
                    pis[pi_index].tvec = tvec
                    pis[pi_index].update_derived_params()
                else:
                    print("Could not solvePnP, no idea why")
            else:
                display_frames.append(frames[pi_index])
                all_found = False

        joined_frame = np.concatenate((display_frames[0], display_frames[1]), axis=1)
        cv2.imshow("frame", joined_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("\nPi0 rvec:\n", pis[0].rvec,
          "\nPi0 tvec:\n", pis[0].tvec,
          "\nPi1 rvec:\n", pis[1].rvec,
          "\nPi1 tvec:\n", pis[1].tvec)
    print("Distance between cameras:", np.linalg.norm(pis[0].tvec - pis[1].tvec))
    print("Distance to calibration rig:", np.linalg.norm(pis[0].tvec))
    cv2.waitKey(0)


def detect_chessboard(frame):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
    objp = objp * CHESSBOARD_SIZE

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    # If found, add object points, image points (after refining them)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Draw and display the corners
        frame_with_corners = cv2.drawChessboardCorners(frame, (7, 6), corners2, ret)

        return True, objp, corners2, frame_with_corners
    else:
        return False, None, None, None


def calibrate_intrinsic(pi):
    """This function is not being used and is not in development"""
    CALIB_FRAMES = 10
    calib_frame_count = 0
    cam_mtx_list = []
    while calib_frame_count < CALIB_FRAMES:
        frame, timestamp, quick_read = pi.get_frame()

        time1 = time.time()
        ret, objpoints, imgpoints, frame_with_corners = detect_chessboard(frame)
        print("Detecting chessboard took:", time.time() - time1)

        if ret:
            ret, cam_mtx, dist, rvec, tvec = cv2.calibrateCamera(objpoints, imgpoints, frame.shape[1::-1], None, None,
                                                                 flags=(cv2.CALIB_ZERO_TANGENT_DIST
                                                                        + cv2.CALIB_FIX_K1
                                                                        + cv2.CALIB_FIX_K2
                                                                        + cv2.CALIB_FIX_K3))
            cam_mtx_list.append(cam_mtx)
            np.set_printoptions(formatter={'float': "{0:0.3f}".format})
            print("Camera matrix:\n", cam_mtx)
            # print("Distortion coeffs:\n", dist)
            # print("Rotation vector:\n", rvecs)
            print("Translation vector:\n", tvec)
            calib_frame_count += 1
            output_frame = frame_with_corners
        else:
            output_frame = frame

        cv2.imshow("chessboard", output_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("cam_mtx_list:", cam_mtx_list)
    cam_mtx_median = np.median(cam_mtx_list, axis=0)
    print("cam_mtx_median:", cam_mtx_median)
    pi.K = cam_mtx_median
