import cv2
import numpy as np
import time
import frame

CHESSBOARD_SIZE = 2.42  # real-life size of a square in cm


def calibrate_extrinsic_correspondences(pisys):
    np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

    def draw_correspondence_markers(original_frame, correspondences, colours):
        """Adds markers for each of these correspondences to drawing_frame"""
        drawing_frame = original_frame.copy()
        for id in range(len(correspondences)):
            for n in range(len(correspondences[id])):
                drawing_frame.add_marker(id, correspondences[id][n], n, colours[n])
        return drawing_frame

    def click_event(event, x, y, flags, param):
        nonlocal original_frame, drawing_frame, correspondences, order_added, colours

        if event == cv2.EVENT_LBUTTONDOWN:
            sub_x, sub_y, id = drawing_frame.locate_in_combined(x, y)

            n = len(correspondences[id])
            if n == len(min(correspondences, key=len)):
                correspondences[id].append((sub_x, sub_y))
                order_added.append(id)
                drawing_frame = draw_correspondence_markers(original_frame, correspondences, colours)
            else:
                print("Please select matching correspondence on other image")

    original_frame = pisys.get_synced_multiframe()
    drawing_frame = original_frame.copy()

    correspondences, order_added = [[], []], []  # correpondences[0] is a list of lists for Pi0
    num_correspondences = 10  # 7 is theoretical minimum
    colours = np.random.rand(num_correspondences, 3) * 255
    # real_distance = float(input("Enter distance in cm between the first two points you select:"))
    real_distance = 30
    print("Please select correspondances. Controls: u = undo, n = next frame, c = clear frame, q = quit")

    if False:  # This if statement can eventually be deleted
        cv2.namedWindow('Select correspondences')
        cv2.setMouseCallback('Select correspondences', click_event)
        while len(min(correspondences, key=len)) < num_correspondences:
            cv2.imshow('Select correspondences', drawing_frame.combine_frames())
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                # Quit
                return
            elif key == ord('n'):
                # New frame
                original_frame = pisys.get_synced_multiframe()
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
    else:
        # Problematic dataset
        correspondences = [
            [(282, 660), (692, 716), (951, 272), (1391, 412), (1636, 404), (622, 647), (702, 842), (854, 225),
             (1339, 119), (830, 386)],
            [(552, 773), (1020, 619), (257, 287), (800, 320), (951, 270), (708, 646), (1558, 587), (80, 255), (663, 62),
             (115, 469)]]
        drawing_frame = draw_correspondence_markers(original_frame, correspondences, colours)

    # Calculate external params from correspondences
    # This part assumes both cameras have the same camera matrix. Use cv2.undistortPoints to improve
    points = np.array(correspondences).astype(float)
    print("Correspondences:\n", correspondences)
    F, F_mask = cv2.findFundamentalMat(points[0], points[1], method=cv2.FM_8POINT)
    print("F:\n", F)
    if not np.all((F_mask == 1)):
        print("Warning: some correspondences were rejected as outliers. Mask:\n", F_mask)

    points_homo = cv2.convertPointsToHomogeneous(points.reshape((-1, 2))).reshape((2, len(points[0]), 3))
    error = np.diagonal(np.matmul(np.matmul(points_homo[1], F), points_homo[0].T)).reshape((-1, 1))
    print("Error in F:\n", error)

    K = pisys.pis[0].K
    E = np.matmul(K.T, np.matmul(F, K))
    retval, R, tvec, pose_mask = cv2.recoverPose(E, points[0], points[1], K)
    rvec, jac = cv2.Rodrigues(R)

    pisys.pis[0].set_extrinsic_params(np.array([[0.0], [0.0], [0.0]]), np.array([[0.0], [0.0], [0.0]]))
    # tvec += [[0], [-0.03], [-0.1]]
    pisys.pis[1].set_extrinsic_params(rvec, tvec)

    # Calculate 3D position of correspondences and calibrate for scale using fist two points
    points3D = drawing_frame.triangulate_points(points)
    old_distance = np.linalg.norm(points3D[0, 0] - points3D[1, 0])
    factor = real_distance / old_distance
    points3D *= factor
    for pi in pisys.pis:
        new_tvec = pi.tvec * factor
        pi.set_extrinsic_params(pi.rvec, new_tvec)

    # Output info messages
    print("Correspondences in 3D:\n", points3D.reshape((-1, 3)))
    print("Real tvec:\n", pisys.pis[1].tvec)
    print("Distance between cameras:", np.linalg.norm(pisys.pis[1].tvec))

    drawing_frame.draw_axes(points3D)
    cv2.imshow('Select correspondences', drawing_frame.combine_frames())
    cv2.waitKey(0)


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
