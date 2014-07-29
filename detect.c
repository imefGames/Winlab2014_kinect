// Be sure to link with -lfreenect_sync -lm -pthread
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <libfreenect_sync.h>
#include <pthread.h>
#include "kinectDetectionUtil.h"

#define BUFLEN 8
#define PORT 5005

///prototypes
//packet
void writePacket(char* packet, char type, short data1, short data2, short data3);
//thread
void *readAsync(void *threadid);

///global variables
int contLoop;

///functions
int main(int argc, char* argv[])
{
	//input parametres
	if(argc != 2){
		printf("usage: %s <ip>\n", argv[0]);
        return EXIT_FAILURE;
	}
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
	//set UDP socket
	struct sockaddr_in si_other;
	int s, i, slen=sizeof(si_other);
	char buf[BUFLEN];
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
		fprintf(stderr, "socket() failed\n");
		return 1;
	}
	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	if (inet_aton(argv[1], &si_other.sin_addr)==0) {
		fprintf(stderr, "inet_aton() failed\n");
		return 1;
	}
	//set cameras
	TDepthCamera mainCam, secCam;
	createPrimaryCamera(&mainCam, 0);
	createPrimaryCamera(&secCam, 1);
	FILE* pFile = NULL;
	pFile = fopen("calibrationValues.cal", "r");
	if(pFile == NULL){
		puts("Could not get calibration data.");
	}else{
		fread(&minZ, sizeof(int), 1, pFile);
		fread(&maxZ, sizeof(int), 1, pFile);
		fread(secCam.base, sizeof(TMatrix4D), 1, pFile);
	}
	TVecList mainList, secList;
	unsigned int timestamp;
	contLoop = 1;
	//show current calibration values.
	printf("Current calibration values:\nCeiling: %d, Floor: %d\nTransformation matrix:\n", maxZ, minZ);
	displayMatrix4(secCam.base);
	puts("\n\nAre those values correct? [Y/N]");
	char tmpChar = getchar();
	if(tmpChar == 'N' || tmpChar == 'n'){
        contLoop = 0;
        puts("\nUse calibration program to correct the values.");
	}
	fflush(stdin);
	//start thread
	pthread_t thread;
	int rc;
	long t = 0;
	rc = pthread_create(&thread, NULL, readAsync, (void *)t);
	if (rc){
		printf("ERROR; return code from pthread_create() is %d\n", rc);
		exit(-1);
	}
	//main loop
	while(contLoop){
		//acquire data for main kinect & process data
		if(updateCamera(&mainCam, &timestamp)){
            printf("Could not update feed for device 0.");
            return EXIT_FAILURE;
		}
		if(detectDrone(mainCam.data, &mainList, &vec3DDistance)){
            printf("Could not process data for for device 0.");
            return EXIT_FAILURE;
		}
		//acquire data for secondary kinect & process data
		if(updateCamera(&secCam, &timestamp)){
            printf("Could not update feed for device 1.");
            return EXIT_FAILURE;
		}
		if(detectDrone(secCam.data, &secList, &vec3DDistance)){
            printf("Could not process data for for device 1.");
            return EXIT_FAILURE;
		}
		//convert main points to secondary base
		int i;
		for(i=0; i<secList.n; i++){
            transformVec4D(&(secList.vector[i]), secCam.base);
		}
		//match both lists
		fusePointList(&mainList, &secList, 200, &vec3DDistance);
		simplifyPointList(&mainList, 200, &vec3DDistance);
		//display list
		system("clear");
		puts("Press Enter to exit.\n\n---------------\nLIST:");
		displayVecList(&mainList);
		//send command to drone
		//send position to unity3D
		TVec4D* maxVect = maxPointList(&mainList);
		if(maxVect != NULL){
            writePacket(buf, 'k', maxVect->x, maxVect->y, maxVect->z);
			if (sendto(s, buf, BUFLEN, 0, &si_other, slen)==-1){
				fprintf(stderr, "sendto() failed\n");
				return 1;
			}
		}
	}
	//close socket
	close(s);
	//free all data
	freeCamera(&mainCam);
	freeCamera(&secCam);
	//stop kinects
	freenect_sync_stop();
	//stop pthread
	pthread_exit(NULL);
	return EXIT_SUCCESS;
}

//packet
void writePacket(char* packet, char type, short data1, short data2, short data3){
	int i;
	char crc8 = type;
	packet[0] = type;
	*((short*)&packet[1]) = data1;
	*((short*)&packet[3]) = data2;
	*((short*)&packet[5]) = data3;
	for(i=1; i<7; i++){
		crc8 += packet[i];
	}
	packet[7] = crc8;
}

//thread
void *readAsync(void *threadid)
{
   getchar();
   contLoop = 0;
   pthread_exit(NULL);
}
