import cv2
import numpy as np


class Calibrator:
    def __init__(self, pisys):
        self.pisys = pisys
        self.original_frame = self.pisys.get_synced_multiframe()
        self.drawing_frame = self.original_frame.copy()
        self.correspondences, self.order_added = [[], []], []  # correpondences[0] is a list of lists for Pi0
        self.correspondences_3D = None
        self.colours = np.random.rand(100, 3) * 255  # 100 is arbitrary
        self.method = cv2.FM_8POINT  # Default method
        self.distance_first_2 = float(input("Enter distance in cm between the first two points you select:\n"))
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
        # print("Correspondences:\n", correspondences)
        # Calculate external params from correspondences
        # This part assumes both cameras have the same camera matrix. Use cv2.undistortPoints to improve
        points = np.array(correspondences).astype(float)
        F, F_mask = cv2.findFundamentalMat(points[0], points[1], method=self.method)

        # Evaluating the effectiveness of the F matrix
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
              "Controls: q = quit without saving, f = finish, u = undo, n = next frame,\n",
              "          c = clear frame, d = delete point, m = toggle method")

        cv2.namedWindow('Select correspondences')
        cv2.setMouseCallback('Select correspondences', self.click_event)
        while True:  # len(min(self.correspondences, key=len)) < self.num_correspondences:
            cv2.imshow('Select correspondences', self.drawing_frame.combine_frames())
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                # Quit without saving
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
