//Compiler instructions for one kinect
gcc calibrateOneKinect.c kinectDetectionUtil.c -o calibrateOne -lm -lfreenect_sync;
gcc detectOneKinect.c kinectDetectionUtil.c -o detectOne -lm -lfreenect_sync -pthread

//Compiler instructions for two kinects
gcc calibrate.c kinectDetectionUtil.c -o calibrate -lm -lfreenect_sync;
gcc detect.c kinectDetectionUtil.c -o detect -lm -lfreenect_sync -pthread;


//Compiler instructions for one kinect to 2 IPs
gcc detectOneKinect2IP.c kinectDetectionUtil.c -o detectOne2IP -lm -lfreenect_sync -pthread

//Compiler instructions for two kinects to IPs
gcc detect2IP.c kinectDetectionUtil.c -o detect2IP -lm -lfreenect_sync -pthread;


