import cv2
import numpy as np
import time
import video

CHESSBOARD_SIZE = 2.42  # real-life size of a square in cm


def calibrate_extrinsic_correspondences(pis):
    np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})

    def click_event(event, x, y, flags, param):
        nonlocal original_frame, drawing_frame, correspondences, order_added, colours

        if event == cv2.EVENT_LBUTTONDOWN:
            if x < drawing_frame.shape[1] / 2:
                # Point selected on Pi0 image
                id = 0
                sub_x, sub_y = x, y
            else:
                id = 1
                sub_x, sub_y = x - drawing_frame.shape[1]//2, y

            n = len(correspondences[id])
            if n == len(min(correspondences, key=len)):
                correspondences[id].append((sub_x, sub_y))
                order_added.append(id)
                drawing_frame = draw_correspondence_markers(original_frame, correspondences, colours)
            else:
                print("Please select matching correspondence on other image")

    def draw_correspondence_markers(original_frame, correspondences, colours):
        """Adds markers for each of these correspondences to drawing_frame"""
        drawing_frame = original_frame.copy()
        for i in range(len(correspondences[0])):
            cv2.drawMarker(drawing_frame, correspondences[0][i],
                           colours[i], cv2.MARKER_TILTED_CROSS, markerSize=50, thickness=2)
        for i in range(len(correspondences[1])):
            cv2.drawMarker(drawing_frame,
                           (correspondences[1][i][0] + drawing_frame.shape[1]//2, correspondences[1][i][1]),
                           colours[i], cv2.MARKER_TILTED_CROSS, markerSize=50, thickness=2)
        return drawing_frame

    frames, frame_drop, both_quick_read, avg_latency = video.get_synced_frames(pis)
    original_frame = np.concatenate((frames[0], frames[1]), axis=1)
    drawing_frame = original_frame.copy()
    correspondences, order_added = [[], []], []  # correpondences[0] is a list of lists for Pi0
    num_correspondences = 7  # 7 is theoretical minimum
    colours = np.random.rand(num_correspondences, 3) * 255
    print("Please select correspondances. Controls: u = undo, n = next frame, c = clear frame, q = quit")
    # real_distance = float(input("Enter distance in cm between the first two points you select:"))
    real_distance = 30
    cv2.namedWindow('Select correspondences')
    cv2.setMouseCallback('Select correspondences', click_event)

    while len(min(correspondences, key=len)) < num_correspondences:
        cv2.imshow('Select correspondences', drawing_frame)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('q'):
            # Quit
            break
        elif key == ord('n'):
            # New frame
            frames, frame_drop, both_quick_read, avg_latency = video.get_synced_frames(pis)
            original_frame = np.concatenate((frames[0], frames[1]), axis=1)
            drawing_frame = original_frame.copy()
            correspondences, order_added = [[], []], []
        elif key == ord('c'):
            # Clear correspondences
            drawing_frame = original_frame.copy()
            correspondences, order_added = [[], []], []
        elif key == ord('u'):
            # Undo last correspondence
            try:
                correspondences[order_added[-1]].pop()
                order_added.pop()
                drawing_frame = original_frame.copy()
                drawing_frame = draw_correspondence_markers(original_frame, correspondences, colours)
            except IndexError:
                pass
    cv2.setMouseCallback('Select correspondences', lambda *args: None)

    # Calculate external params from correspondences
    # This part assumes both cameras have the same camera matrix. Use cv2.undistortPoints to improve
    points = np.array(correspondences).astype(float)
    print("Correspondences:\n", correspondences)
    E, E_mask = cv2.findEssentialMat(points[0], points[1], pis[0].cam_mtx, method=cv2.RANSAC, prob=0.99, threshold=100)
    retval, R, tvec, pose_mask = cv2.recoverPose(E, points[0], points[1], pis[0].cam_mtx, mask=E_mask)
    rvec, jac = cv2.Rodrigues(R)
    pis[0].set_extrinsic_params(np.array([[0.0], [0.0], [0.0]]), np.array([[0.0], [0.0], [0.0]]))
    pis[1].set_extrinsic_params(rvec, tvec)
    if not np.all((E_mask == 1)):
        print("Warning: some correspondences were rejected as outliers. Mask:\n", E_mask)

    # Calculate 3D position of correspondences and calibrate for scale using fist two points
    points4D = cv2.triangulatePoints(pis[0].P, pis[1].P, points[0].T, points[1].T).T
    points3D = cv2.convertPointsFromHomogeneous(points4D)
    old_distance = np.linalg.norm(points3D[0, 0]-points3D[1, 0])
    factor = real_distance/old_distance
    points3D *= factor
    for pi in pis:
        new_tvec = pi.tvec * factor
        pi.set_extrinsic_params(pi.rvec, new_tvec)
    print("Correspondences in 3D:\n", points3D)
    print("Real tvec:\n", pis[1].tvec)
    print("Distance between cameras:", np.linalg.norm(pis[1].tvec))

    def draw_axes(drawing_frame, points3D):
        for point in points3D:
            axis = point[0] + np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]])

            # For pi0
            img_points, jac = cv2.projectPoints(axis, pis[0].rvec, pis[0].tvec, pis[0].cam_mtx, distCoeffs=None)
            draw_axes_as_lines(drawing_frame, img_points)

            # For pi1
            img_points, jac = cv2.projectPoints(axis, pis[1].rvec, pis[1].tvec, pis[1].cam_mtx, distCoeffs=None)
            img_points += np.array([drawing_frame.shape[1]//2, 0])
            draw_axes_as_lines(drawing_frame, img_points)

    def draw_axes_as_lines(drawing_frame, img_points):
        img_points = img_points.astype(int)
        drawing_frame = cv2.line(drawing_frame, tuple(img_points[0, 0]), tuple(img_points[1, 0]), (255, 0, 0), 2)
        drawing_frame = cv2.line(drawing_frame, tuple(img_points[0, 0]), tuple(img_points[2, 0]), (0, 255, 0), 2)
        drawing_frame = cv2.line(drawing_frame, tuple(img_points[0, 0]), tuple(img_points[3, 0]), (0, 0, 255), 2)

    draw_axes(drawing_frame, points3D)
    cv2.imshow('Select correspondences', drawing_frame)
    cv2.waitKey(0)


def calibrate_extrinsic_chessboard(pis):
    all_found = False
    while not all_found:
        display_frames = []

        # Get and show new frames
        frames, frame_drop, both_quick_read, avg_latency = video.get_synced_frames(pis)

        # Search for chessboards and attempt to locate them
        all_found = True
        for pi_index in range(len(pis)):
            ret, objpoints, imgpoints, frame_with_corners = detect_chessboard(frames[pi_index])
            if ret:
                display_frames.append(frame_with_corners)
                ret, rvec, tvec, inliers = cv2.solvePnPRansac(objpoints, imgpoints,
                                                              pis[pi_index].cam_mtx, distCoeffs=None)
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
    print("Distance between cameras:", np.linalg.norm(pis[0].tvec-pis[1].tvec))
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
        print("Detecting chessboard took:", time.time()-time1)

        if ret:
            ret, cam_mtx, dist, rvec, tvec = cv2.calibrateCamera(objpoints, imgpoints, frame.shape[1::-1], None, None,
                                                               flags=(cv2.CALIB_ZERO_TANGENT_DIST
                                                                      + cv2.CALIB_FIX_K1
                                                                      + cv2.CALIB_FIX_K2
                                                                      + cv2.CALIB_FIX_K3))
            cam_mtx_list.append(cam_mtx)
            np.set_printoptions(formatter={'float': "{0:0.3f}".format})
            print("Camera matrix:\n", cam_mtx)
            #print("Distortion coeffs:\n", dist)
            #print("Rotation vector:\n", rvecs)
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
    pi.cam_mtx = cam_mtx_median