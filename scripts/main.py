import numpy as np
import cv2
import time

from camera import Camera
from arm import Gen3Lite
from aruco import ArucoDetector, ArucoMarker
import maths


# Hand-eye Calibration Data
R_0_E = maths.quat2dcm([-0.0246767, 0.968246, -0.248064, 0.0188425])
P_0_E = np.array([0.378836, 0.380062, 0.423619])
T_0_E = maths.RP2T(R_0_E, P_0_E)


arm = Gen3Lite()
cam = Camera()
detector = ArucoDetector(cv2.aruco.DICT_ARUCO_ORIGINAL)
marker = ArucoMarker(1)


try:
    # arm.init()
    # arm.move_home()
    # arm.set_gripper_position(1.0)

    cam.init()
    
    while True:
        frame = cam.get_color_frame()
        
        corners, ids, rvecs, tvecs, markerPoints = detector.detect(frame, 0.05, cam.K, cam.Kcoeffs)
        if ids is not None:
            for idx in range(len(ids)):
                id, rvec, tvec = ids[idx][0], rvecs[idx][0], tvecs[idx][0]
                if id == marker.id:
                    marker.update(rvec, tvec)

            pose = marker.get_pose()
            if pose is not None:
                R_E_P, P_E_P = maths.rotvec2dcm(pose[0:3]), pose[3:6]
                T_E_P = maths.RP2T(R_E_P, P_E_P)
                T_0_P = np.dot(T_0_E, T_E_P)
                
                X_P_target = np.array([0.1, -0.1, 0.2])
                X_0_target = np.dot(T_0_P, np.hstack((X_P_target, [1])))[0:3]
                V_0_target = np.array([0, 0, 0])
                
                
                
                # arm.set_cartesian_pose(maths.col2row(X_0_target).tolist() + [90, 0, 160])
                
        
        cv2.imshow('image', detector.draw(frame, corners, rvecs, tvecs, cam.K, cam.Kcoeffs))
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
finally:
    cam.close()
    cv2.destroyAllWindows()
    
    # arm.move_home()
    # arm.stop_motioning()
    # arm.close()


# def delay(t):
#     delay_mark = time.time()    
#     while True:
#         offset = time.time() - delay_mark
#         if offset > t:
#             break


# def draw_fs_trajectory(arm, V = 0.02, R = 0.05, Dphi = 5):
#     def take_updown(arm, fac):
#         arm.set_cartesian_pose(arm.get_current_cartesian_pose() + fac * np.array([0, 0, 0.12, 0, 0, 0]))
    
#     WRITE_READY_POSE = [0.33, 0.088, 0.14, 90, 0, 140]
    
#     try:
#         arm.set_gripper_position(1.0)
#         arm.set_cartesian_pose(WRITE_READY_POSE)
#         take_updown(arm, -1)
#         phi = 0
#         while True:
#             v_x = V * np.cos(phi * np.pi / 180)
#             v_y = -V * np.sin(phi * np.pi / 180)
#             arm.set_twist([v_x, v_y, 0, 0, 0, 0])
#             phi = phi + Dphi
#             Dt = Dphi * np.pi / 180 / V * R
#             # print(Dt)   # Dt 太小会延时不精确导致偏移, e.g. 半径 0.05 m 的圆, Dt 在 300 ms 以上时尺度较准确.
#             delay(Dt)
#     finally:
#         take_updown(arm, 1)