// Be sure to link with -lfreenect_sync -lm
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
	if(freenect_sync_set_tilt_degs(0, 1)){
        printf("Could not tilt device 1.\n");
        return EXIT_FAILURE;
	}
	if(freenect_sync_set_led(LED_YELLOW, 1)){
       	printf("Could not change LED of device 1.\n");
        return EXIT_FAILURE;
	}
	//set cameras
	nbIterations = 200000;
	TDepthCamera mainCam, secCam;
	createPrimaryCamera(&mainCam, 0);
	createPrimaryCamera(&secCam, 1);
	TVecList mainList, secList;
	TVec4D mainPoints[4], secPoints[4];
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
		    //get the depth map + get floor and ceiling
		    int i;
		    updateCamera(&mainCam, &timestamp);
		    detectDrone(mainCam.data, &mainList, &vecHeightDifference);
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
		    updateCamera(&secCam, &timestamp);
		    detectDrone(secCam.data, &secList, &vecHeightDifference);
		    for(i=0; i<secList.n; i++){
                if(secList.vector[i].z >= 0){
                    if(secList.vector[i].z < maxZ){
                        maxZ = secList.vector[i].z;
                    }
                }else{
                    if(secList.vector[i].z > minZ){
                        minZ = secList.vector[i].z;
                    }
                }
		    }
		    puts("\nEnvironment captured.\n");
		    //display values
		    printf("Ceiling: %d, Floor: %d\n\n", maxZ, minZ);
		    puts("\nCapture again? [Y/N] ");
		    k = getchar();
		}while(k == 'y' || k == 'Y');
		minZ += 100;
		maxZ -= 100;
		//capture point 0
		do{
			system("clear");
			puts("Calibration:\nStep 2: Kinect Position\n");
		    puts("\nPlace a thin but tall object within view of both Kinects.\n\n");
		    puts("Press any key when you wish to capture the environment.\n");
		    getchar();
		    //get the depth map + get P0
		    updateCamera(&mainCam, &timestamp);
		    detectDrone(mainCam.data, &mainList, &vec2DDistance);
		    updateCamera(&secCam, &timestamp);
		    detectDrone(secCam.data, &secList, &vec2DDistance);
		    puts("\nEnvironment captured.\n");
		    //display values
		    if(getMaxVectorFromList(&(mainPoints[0]), &mainList)){
                puts("Failed to acquire a point for main camera.\n");
		    }else{
                puts("main P0:");
                displayVec4(&(mainPoints[0]));
		    }
		    if(getMaxVectorFromList(&(secPoints[0]), &secList)){
                puts("Failed to acquire a point for secondary camera.\n");
		    }else{
                puts("\n\nsec P0:");
                displayVec4(&(secPoints[0]));
		    }
		    puts("\nCapture again? [Y/N] ");
		    k = getchar();
		}while(k == 'y' || k == 'Y');
		//capture point 1
		do{
		    system("clear");
			puts("Move the object by about a meter in any direction.\nThe object must remain within view of both Kinects\n\n");
		    puts("Press any key when you wish to capture the environment.\n");
		    getchar();
		    //get the depth map + get P1
		    updateCamera(&mainCam, &timestamp);
		    detectDrone(mainCam.data, &mainList, &vec2DDistance);
		    updateCamera(&secCam, &timestamp);
		    detectDrone(secCam.data, &secList, &vec2DDistance);
		    puts("\nEnvironment captured.\n");
		    //display values
		    if(getMaxVectorFromList(&(mainPoints[1]), &mainList)){
                puts("Failed to acquire a point for main camera.\n");
		    }else{
                puts("main P1:");
                displayVec4(&(mainPoints[1]));
		    }
		    if(getMaxVectorFromList(&(secPoints[1]), &secList)){
                puts("Failed to acquire a point for secondary camera.\n");
		    }else{
                puts("\n\nsec P1:");
                displayVec4(&(secPoints[1]));
		    }
		    //display values
		    puts("\nCapture again? [Y/N] ");
		    k = getchar();
		}while(k == 'y' || k == 'Y');
		//capture point 2
		do{
		    system("clear");
			puts("Move the object by about a meter in an other direction.\nThe object must remain within view of both Kinects\n\n");
		    puts("Press any key when you wish to capture the environment.\n");
		    getchar();
		    //get the depth map + get P2
		    updateCamera(&mainCam, &timestamp);
		    detectDrone(mainCam.data, &mainList, &vec2DDistance);
		    updateCamera(&secCam, &timestamp);
		    detectDrone(secCam.data, &secList, &vec2DDistance);
		    puts("\nEnvironment captured.\n");
		    //display values
		    if(getMaxVectorFromList(&(mainPoints[2]), &mainList)){
                puts("Failed to acquire a point for main camera.\n");
		    }else{
                puts("main P2:\n");
                displayVec4(&(mainPoints[2]));
		    }
		    if(getMaxVectorFromList(&(secPoints[2]), &secList)){
                puts("Failed to acquire a point for secondary camera.\n");
		    }else{
                puts("\n\nsec P2:");
                displayVec4(&(secPoints[2]));
		    }
		    //display values
		    puts("\nCapture again? [Y/N] ");
		    k = getchar();
		}while(k == 'y' || k == 'Y');
		//adjust P1 and P2
		mainPoints[1].z = mainPoints[0].z;
		mainPoints[2].z = mainPoints[0].z;
		secPoints[1].z = secPoints[0].z;
		secPoints[2].z = secPoints[0].z;
		//calculate P3
		mainPoints[3].x = mainPoints[0].x;
		mainPoints[3].y = mainPoints[0].y;
		mainPoints[3].z = mainPoints[0].z + 1000;
		mainPoints[3].w = mainPoints[0].w;
		secPoints[3].x = secPoints[0].x;
		secPoints[3].y = secPoints[0].y;
		secPoints[3].z = secPoints[0].z + 1000;
		secPoints[3].w = secPoints[0].w;
		//get M0 and M1
		TMatrix4D M0, M1, invM1, transformMatrix;
		matrix4DGetFromVectors(&M0, mainPoints);
		matrix4DGetFromVectors(&M1, secPoints);
		//invert M1
        if(matrix4DInvert(&invM1, &M1)){
            puts("Failed to create transformation matrix.");
        }else{
            //get transformation matrix
            matrix4DMultiply(&transformMatrix, &M0, &invM1);
            //display matrix
            puts("Transformation Matrix:\n");
            displayMatrix4(&transformMatrix);
			FILE* pFile = NULL;
			pFile = fopen("calibrationValues.cal", "w");
			if(pFile != NULL){
                fwrite(&minZ, sizeof(int), 1, pFile);
                fwrite(&maxZ, sizeof(int), 1, pFile);
				fwrite(&transformMatrix, sizeof(TMatrix4D), 1, pFile);
				fclose(pFile);
				puts("Calibration data saved.");
			}
        }
		puts("\nCalibrate again? [Y/N] ");
		exLoop = getchar();
	    }while(exLoop == 'y' || exLoop == 'Y');

	//free data
	freeCamera(&mainCam);
	freeCamera(&secCam);
	//stop kinects
	freenect_sync_stop();
	return EXIT_SUCCESS;
}
