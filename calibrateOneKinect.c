#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <libfreenect_sync.h>
#include "kinectDetectionUtil.h"

///functions
int main()
{
	//set kinect angles to 0° & set LED color
	if(freenect_sync_set_tilt_degs(0, 0)){
        printf("Could not tilt device 0.\n");
        return EXIT_FAILURE;
	}
	if(freenect_sync_set_led(LED_GREEN, 0)){
        printf("Could not change LED of device 0.\n");
        return EXIT_FAILURE;
	}
	//increase precision for detection
	nbIterations = 200000;
	//set cameras
	TDepthCamera mainCam;
	createPrimaryCamera(&mainCam, 0);
	TVecList mainList;
	int timestamp;
	char exLoop;
	///calibration
	do{
		//Ground and ceiling
		char k;
		do{
			system("clear");
			puts("Calibration:\nStep 1: Environment\nPlace the Kinects at the desired locations.\nBe careful that nothing is within view of both devices.\n\n");
		    puts("Press any key when you wish to capture the environment.\n");
		    getchar();
		    //get the depth map + get floor and ceiling for main camera
		    int i;
		    updateCamera(&mainCam, &timestamp);
		    detectDrone(mainCam.data, &mainList, &vecHeightDifference);
			//for all points in main list, check if max or min.
		    for(i=0; i<mainList.n; i++){
                if(mainList.vector[i].z >= 0){
                    if(mainList.vector[i].z < maxZ){
                        maxZ = mainList.vector[i].z;
                    }
                }else{
                    if(mainList.vector[i].z > minZ){
                        minZ = mainList.vector[i].z;
                    }
                }
		    }
		    //display values
		    puts("\nEnvironment captured.\n");
		    printf("Ceiling: %d, Floor: %d\n\n", maxZ, minZ);
			//suggest new capture
		    puts("\nCapture again? [Y/N] ");
		    k = getchar();
		}while(k == 'y' || k == 'Y');
		//adjust min and max for safety
		minZ += 100;
		maxZ -= 100;
		FILE* pFile = NULL;
		pFile = fopen("calibrationValuesOne.cal", "w");
		if(pFile != NULL){
			fwrite(&minZ, sizeof(int), 1, pFile);
			fwrite(&maxZ, sizeof(int), 1, pFile);
			fclose(pFile);
			puts("Calibration data saved.");
		}
		//suggest new calibration
		puts("\nCalibrate again? [Y/N] ");
		exLoop = getchar();
	}while(exLoop == 'y' || exLoop == 'Y');
	//free data
	freeCamera(&mainCam);
	//stop kinects
	freenect_sync_stop();
	return EXIT_SUCCESS;
}
