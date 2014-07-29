gcc calibrate.c kinectDetectionUtil.c -o calibrate -lm -lfreenect_sync;
gcc detect.c kinectDetectionUtil.c -o detect -lm -lfreenect_sync -pthread;