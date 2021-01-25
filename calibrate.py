import cv2
import numpy as np
import time
import video

CHESSBOARD_SIZE = 2.42  # real-life size of a square in cm


def calibrate_extrinsic_correspondences(pis):
    def click_event(event, x, y, flags, param):
        nonlocal drawing_frame, correspondences, colours

        if event == cv2.EVENT_LBUTTONDOWN:
            if x < drawing_frame.shape[1] / 2:
                # Point selected on Pi0 image
                id = 0
                sub_x, sub_y = x, y
            else:
                id = 1
                sub_x, sub_y = x - drawing_frame.shape[1]/2, y

            n = len(correspondences[id])
            if n == len(min(correspondences, key=len)):
                correspondences[id].append([sub_x, sub_y])
                cv2.drawMarker(drawing_frame, (x, y), colours[n], cv2.MARKER_TILTED_CROSS, markerSize=50, thickness=1)
            else:
                print("Please select matching correspondence on other image")

    frames, frame_drop, both_quick_read, avg_latency = video.get_synced_frames(pis)
    original_frame = np.concatenate((frames[0], frames[1]), axis=1)
    drawing_frame = original_frame.copy()
    correspondences = [[], []]  # correpondences[0] is a vector of vectors for Pi0
    num_correspondences = 11 #7
    colours = np.random.rand(num_correspondences, 3) * 255
    real_distance = 30
    #real_distance = float(input("Enter distance in cm between the first two points you select:"))
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
            correspondences = [[], []]
        elif key == ord('c'):
            # Clear correspondences
            drawing_frame = original_frame.copy()
            correspondences = [[], []]

    # This part assumes both cameras have the same camera matrix. Use cv2.undistortPoints to improve
    points = np.array(correspondences)
    E, E_mask = cv2.findEssentialMat(points[0], points[1], pis[0].cam_mtx, mask=[[1]*num_correspondences])
    retval, R, tvec, pose_mask = cv2.recoverPose(E, points[0], points[1], pis[0].cam_mtx, mask=E_mask)
    rvec, jac = cv2.Rodrigues(R)
    pis[0].rvec = np.array([[0.0], [0.0], [0.0]])
    pis[0].tvec = np.array([[0.0], [0.0], [0.0]])
    pis[0].update_derived_params()
    pis[1].rvec = rvec
    pis[1].tvec = tvec
    pis[1].update_derived_params()

    print("E matrix points mask:\n", E_mask)
    print("Pose mask\n", pose_mask)
    print("Essential matrix found:\n", E)
    print("Normalised tvec:\n", tvec)
    print("rvec:\n", rvec)

    points4D = cv2.triangulatePoints(pis[0].P, pis[1].P, points[0].T, points[1].T).T
    points3D = cv2.convertPointsFromHomogeneous(points4D)
    old_distance = np.linalg.norm(points3D[0,0]-points3D[1,0])
    factor = real_distance/old_distance
    points3D *= factor
    for pi in pis:
        pi.tvec *= factor
        pi.update_derived_params()
    print(points3D)
    print("Real tvec:\n", pis[1].tvec)
    print("Distance between cameras:", np.linalg.norm(pis[1].tvec))


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