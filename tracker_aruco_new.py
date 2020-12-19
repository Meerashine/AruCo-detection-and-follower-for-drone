# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 19:40:04 2020

@author: Meerashine Joe
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import pickle
import os

def arucoTracker(cap):
# Check for camera calibration data
    if not os.path.exists('./calibration2.pckl'):
       print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
       exit()
    else:
       f = open('calibration2.pckl', 'rb')
       (ret, mtx, dist, rvecs, tvecs,) = pickle.load(f)
       f.close()
    if mtx is None or dist is None:
        print("Calibration issue. Remove ./calibration2.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()
    #cap = cv2.VideoCapture(0)


###------------------ ARUCO TRACKER ---------------------------
    while (True):
        #et, frame = cap.read()
        detected_ids = {}
    # operations on the frame
        gray = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)

    # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

    # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crash
        if np.all(ids != None):

        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
            rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
            (rvec-tvec).any()  # get rid of that nasty numpy value array error

            for i in range(0, ids.size):
            # draw axis for the aruco markers
               aruco.drawAxis(cap, mtx, dist, rvec[i], tvec[i], 0.1)

        # draw a square around the markers
            aruco.drawDetectedMarkers(cap, corners)
        #print(cv2.norm(tvec), " meters")
            #print(cv2.norm(dist))


        # code to show ids of the marker found
            strg = ''
            for i in range(0, ids.size):
                detected_ids[str(ids[i][0])] = (0,0)
                strg += str(ids[i][0])+', '
                #print(detected_ids)

            cv2.putText(cap, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


        else:
        # code to show 'No Ids' when no markers are found
             cv2.putText(cap, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
   

        return cap, detected_ids
   # display the resulting frame
    
       # cv2.imshow('frame',frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
         #  break
        
   
# When everything done, release the capture
   # cap.release()
    #cv2.destroyAllWindows()
    #print(detected_ids)
   