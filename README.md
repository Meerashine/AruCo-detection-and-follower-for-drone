# AruCo-detection-and-follower-for-drone
The repository consists of program that detects aruco in the vedio frame of tello drone and the drone follows the aruco after detection of the ID.
There are three files in the repo , which consist of a tello library (ibtello.py), aruco detection code( aruco_new.py) and the run file. From the drone we extract the vedio frame 
and will do a detcetion on each frame with a delay of 2 sec and will give the drone commands according to the ID of arucos detected in the frame.
