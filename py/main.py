import numpy as np
import cv2
import time

import maths
from camera import Camera
from arm import Gen3Lite
from aruco import ArucoDetector, ArucoMarker
from control import PIDController


# Hand-eye Calibration Data
R_0_E = maths.quat2dcm([-0.0246767, 0.968246, -0.248064, 0.0188425])
P_0_E = np.array([0.378836, 0.380062, 0.423619])
T_0_E = maths.RP2T(R_0_E, P_0_E)

HOME_X = np.array([0.438, 0.193, 0.447])


arm = Gen3Lite()
cam = Camera()
detector = ArucoDetector(cv2.aruco.DICT_ARUCO_ORIGINAL)
marker = ArucoMarker(1)
pos_controller = PIDController(1.2, 0.03, 0.2, 3)
ang_controller = PIDController(100, 1, -10, 2)

try:
    arm.init()
    arm.move_home()
    arm.set_gripper_position(1.0)

    cam.init()
    
    pos_controller.start()
    ang_controller.start()
    
    lost_cnt = 0
    while True:
        frame = cam.get_color_frame()
        
        corners, ids, rvecs, tvecs, markerPoints = detector.detect(frame, 0.05, cam.K, cam.Kcoeffs)
        if ids is not None:
            lost_cnt = 0
            
            for idx in range(len(ids)):
                id, rvec, tvec = ids[idx][0], rvecs[idx][0], tvecs[idx][0]
                if id == marker.id:
                    marker.update(rvec, tvec)

            rtvecs = marker.get_rtvecs()
            if rtvecs is not None:
                T_E_P = maths.rtvecs2T(rtvecs)
                T_0_P = np.dot(T_0_E, T_E_P)
                
                current_pose = arm.get_current_cartesian_pose()
                current_velocity = arm.get_current_cartesian_velocity()
                
                # kinematics = arm.get_kinematics()
                # if kinematics is None:
                #     continue
                # T_0_H = maths.pose2T(kinematics)
                # T_H_0 = np.linalg.inv(T_0_H)
                T_0_H = maths.pose2T(current_pose)
                T_H_0 = np.linalg.inv(T_0_H)
                
                X_P_target = np.array([0.1, -0.1, 0.12])
                X_H_target = np.dot(T_H_0, np.dot(T_0_P, np.hstack((X_P_target, [1]))))[0:3]
                V_H_target = np.array([0, 0, 0])
                V_H = np.dot(T_H_0[0:3, 0:3], current_velocity[0:3])
                E_X_H = X_H_target
                E_V_H = V_H_target - V_H
                dV_H = pos_controller.step(E_X_H, E_V_H)
                
                R_H_P = np.dot(T_H_0[0:3, 0:3], T_0_P[0:3, 0:3])
                normal_H = R_H_P[:, 2]
                E_normal_H = np.array([normal_H[2], -normal_H[0]])
                dOmega = ang_controller.step(E_normal_H, None)
                
                arm.set_twist(dV_H.tolist() + [dOmega[0], 0, dOmega[1]], 'tool')
        else:
            lost_cnt += 1
            if lost_cnt > 10:
                arm.stop_motioning()
        
        cv2.imshow('image', detector.draw(frame, corners, rvecs, tvecs, cam.K, cam.Kcoeffs))
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
finally:
    cam.close()
    cv2.destroyAllWindows()
    
    arm.move_home()
    arm.stop_motioning()
    arm.close()

