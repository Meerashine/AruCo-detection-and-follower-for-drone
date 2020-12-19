
"""
Created on Thu Dec 17 11:18:33 2020

@author: Meerashine Joe
"""
import ibtello
import cv2 as cv
import time
from tracker_aruco_new import arucoTracker

drone = ibtello.Tello("",8889)

drone.stream_on()
drone.takeoff()
time.sleep(5)

while True:
    frame = drone.read()
    if frame is not None:
        cv.imshow("Tello", frame)
        frame,id =arucoTracker(frame)
        #print(id)
        id_list=list(id.keys())
        print(id_list)
        if len(id_list)==0:
            time.sleep(5)
        elif len(id_list )>= 1:
            if id_list[0]=="1" or id_list[0]=='3' or id_list[0]=='2' or id_list[0]=='4'or id_list[0]=="91" or id_list[0]=='93':
                #print("Move forward list1")
                drone.move_forward(0.5)
            #time.sleep(5)
            elif id_list[0]=='92' or id_list[0]=='94':
                drone.move_right(0.9)
                time.sleep(5)
                drone.move_forward(0.5)
                time.sleep(5)
                drone.land() 
        k = cv.waitKey(1) & 0xFF
        if k == 27:
            break
        
    

