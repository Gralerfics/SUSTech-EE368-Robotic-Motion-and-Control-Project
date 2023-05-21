import numpy as np
import cv2

# from kalman import KalmanFilter


class ArucoDetector:
    def __init__(self, dictionary):
        self.dictionary = cv2.aruco.Dictionary_get(dictionary)
        self.parameters = cv2.aruco.DetectorParameters_create()
    
    
    def detect(self, frame, length, K, Kcoeffs):
        corners, ids, rejects = cv2.aruco.detectMarkers(frame, self.dictionary, parameters = self.parameters, cameraMatrix = K, distCoeff = Kcoeffs)
        rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, length, K, Kcoeffs)
        return corners, ids, rvecs, tvecs, markerPoints
    
    
    def draw(self, frame, corners, rvecs, tvecs, K, Kcoeffs):
        try:
            img_ret = np.copy(frame)
            cv2.aruco.drawDetectedMarkers(img_ret, corners)
            cv2.aruco.drawAxis(img_ret, K, Kcoeffs, rvecs, tvecs, 0.05)
            return img_ret
        except:
            return frame


class ArucoMarker:
    def __init__(self, id):
        self.id = id
        # self.kf = KalmanFilter(0.1, 6, 6)
        self.pose = None


    def update(self, rvec, tvec):
        pose = np.concatenate((rvec, tvec), axis = 0)
        # self.kf.predict()
        # self.kf.update(pose)
        self.pose = pose
        

    def get_pose(self):
        # return self.kf.x
        return self.pose

