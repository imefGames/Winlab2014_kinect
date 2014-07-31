Winlab2014_kinect
=================
Code for the detection of the AR drone in the Summer Internship Program 2014 in Winlab.


calibrateOneKinect.c
====================
Program used to calibrate one Kinect.
Calibration consists in finding the floor and the ceiling of the room.


detectOneKinect.c
=================
Program used to detect the position of an AR drone with one Kinect. This program requires calibration before use.


calibrate.c
===========
Program used to calibrate two Kinects.
Calibration consists in finding the floor and the ceiling of the room.
A transformation matrix is also generated to match the data of both Kinects.


detect.c
========
Program used to detect the position of an AR drone with two Kinects. This program requires calibration before use.


kinectDetectionUtil.c
=====================
C file containing all functions used by the files above.