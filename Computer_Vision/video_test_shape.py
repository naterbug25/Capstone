# Nathan Huber
# Senior Design Capstone
# Rev: 1.0
# Head Tracking with Angles

# activate opencv-env
#!/usr/bin/env python3
import subprocess
import sys
sys.path.append('/Users/hubern/Desktop/Nathan Huber Senior/Senior_Design/') # Windows
sys.path.append('Users/hubern/Desktop/Nathan Huber Senior/head-pose-estimation-master-20190304T182905Z-001/head-pose-estimation-master') # WINDOWS
sys.path.append('/Users/hubern/Desktop/Nathan Huber Senior/Senior_Design/pycomm3') # Windows
sys.path.append('Users/hubern/Desktop/Nathan Huber Senior/head-pose-estimation-master-20190304T182905Z-001/head-pose-estimation-master/dlib-19.6')
sys.path.append('Users/hubern/Downloads')
import SD_Huber_Tag_Handle_20190116

import cv2
import dlib
import numpy as np
from imutils import face_utils

from pycomm.ab_comm.clx import Driver as ClxDriver
import time
import logging
import SD_Huber_Tag_Handle_20190116
import random

face_landmark_path = './shape_predictor_68_face_landmarks.dat'

K = [6.5308391993466671e+002, 0.0, 3.1950000000000000e+002,
     0.0, 6.5308391993466671e+002, 2.3950000000000000e+002,
     0.0, 0.0, 1.0]
D = [7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000]

cam_matrix = np.array(K).reshape(3, 3).astype(np.float32)
dist_coeffs = np.array(D).reshape(5, 1).astype(np.float32)

object_pts = np.float32([[6.825897, 6.760612, 4.402142],
                         [1.330353, 7.122144, 6.903745],
                         [-1.330353, 7.122144, 6.903745],
                         [-6.825897, 6.760612, 4.402142],
                         [5.311432, 5.485328, 3.987654],
                         [1.789930, 5.393625, 4.413414],
                         [-1.789930, 5.393625, 4.413414],
                         [-5.311432, 5.485328, 3.987654],
                         [2.005628, 1.409845, 6.165652],
                         [-2.005628, 1.409845, 6.165652],
                         [2.774015, -2.080775, 5.048531],
                         [-2.774015, -2.080775, 5.048531],
                         [0.000000, -3.116408, 6.097667],
                         [0.000000, -7.415691, 4.070434]])

reprojectsrc = np.float32([[10.0, 10.0, 10.0],
                           [10.0, 10.0, -10.0],
                           [10.0, -10.0, -10.0],
                           [10.0, -10.0, 10.0],
                           [-10.0, 10.0, 10.0],
                           [-10.0, 10.0, -10.0],
                           [-10.0, -10.0, -10.0],
                           [-10.0, -10.0, 10.0]])

line_pairs = [[0, 1], [1, 2], [2, 3], [3, 0],
              [4, 5], [5, 6], [6, 7], [7, 4],
              [0, 4], [1, 5], [2, 6], [3, 7]]


def get_head_pose(shape):
    image_pts = np.float32([shape[17], shape[21], shape[22], shape[26], shape[36],
                            shape[39], shape[42], shape[45], shape[31], shape[35],
                            shape[48], shape[54], shape[57], shape[8]])

    _, rotation_vec, translation_vec = cv2.solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs)

    reprojectdst, _ = cv2.projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix,
                                        dist_coeffs)

    reprojectdst = tuple(map(tuple, reprojectdst.reshape(8, 2)))

    # calc euler angle
    rotation_mat, _ = cv2.Rodrigues(rotation_vec)
    pose_mat = cv2.hconcat((rotation_mat, translation_vec))
    _, _, _, _, _, _, euler_angle = cv2.decomposeProjectionMatrix(pose_mat)

    return reprojectdst, euler_angle


def main():
    # return
    cap = cv2.VideoCapture(0)
    TX_Angle=0
    # Smoothing
    Avg_Threshold = 10 # Image must change by +- this value
    # Averaging 
    Avg = 0	# Average of the X results
    Avg_Sum=[0,0,0,0,0]	# Contains Values to be average
    Avg_Sum_WT=[0.1,0.1,0.25,0.25,0.30]	# Weights of the values to reduce error
    Angle = 0 # Angle to be averaged
    TX_Prev=0# Previous angle sent
    
    if not cap.isOpened():
        print("Unable to connect to camera.")
        return
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(face_landmark_path)

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            face_rects = detector(frame, 0)

            if len(face_rects) > 0:
                shape = predictor(frame, face_rects[0])
                shape = face_utils.shape_to_np(shape)

                reprojectdst, euler_angle = get_head_pose(shape)

                for (x, y) in shape:
                    cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
            try: 
                for start, end in line_pairs:
                    cv2.line(frame, reprojectdst[start], reprojectdst[end], (0, 0, 255))

                cv2.putText(frame, "X: " + "{:7.2f}".format(euler_angle[0, 0]), (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 0, 0), thickness=2)
                cv2.putText(frame, "Y: " + "{:7.2f}".format(euler_angle[1, 0]), (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 0, 0), thickness=2)
                Angle=(euler_angle[1, 0])
                cv2.putText(frame, "Z: " + "{:7.2f}".format(euler_angle[2, 0]), (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 0, 0), thickness=2)
            except:
                print("FAULT.")
            cv2.imshow("demo", frame)
            
               # Running average to reduce error. # Newest to the array goes in the zero place holder of array
            for Avg_Count in range(0,len(Avg_Sum)-1):      # For every spot in the array up to the last place, move all results forward
                Avg_Sum[Avg_Count+1]=Avg_Sum[Avg_Count]       # Shift each one up a spot 
            Avg_Sum[0]=Angle	# Store value in array
            Avg=round(np.average(Avg_Sum, axis=None, weights=Avg_Sum_WT, returned=False))
            print("Avg: {}" .format(Avg))
            # Check and see if face exceeded limits to reduce noise and jitter
            if (((TX_Prev-Avg_Threshold)<Avg<(TX_Prev+Avg_Threshold))): # If the new location is not within the range dont update robot.
                print("Not a big enough change")
            else:
                TX_Angle=Angle
                print("TX_Angle")
                print(TX_Angle)
                SD_Huber_Tag_Handle_20190116.PI_PLC(int(TX_Angle))
                TX_Prev=TX_Angle # Previous angle sent

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    logging.basicConfig(
        filename="ClxDriver.log",
        format="%(levelname)-10s %(asctime)s %(message)s",
        level=logging.DEBUG
    )
    PLC = ClxDriver()

    main()