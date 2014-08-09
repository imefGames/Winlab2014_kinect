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
void writePacket(char* packet, char type, short data1, short data2, short data3);
void *readAsync(void *threadid);

///global variables
int contLoop;

///functions
int main(int argc, char* argv[])
{
	//input parameters
	if(argc != 3){
		printf("usage: %s <first ip> <second ip>\n", argv[0]);
        return EXIT_FAILURE;
	}
	//set Kinect angles to 0° & set LED colour
	if(freenect_sync_set_tilt_degs(0, 0)){
        printf("Could not tilt device 0.\n");
        return EXIT_FAILURE;
	}
	if(freenect_sync_set_led(LED_GREEN, 0)){
        printf("Could not change LED of device 0.\n");
        return EXIT_FAILURE;
	}
	//set UDP socket
	struct sockaddr_in si_other, si_other2;
	int s, i, slen=sizeof(si_other);
	char buf[BUFLEN];
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
		fprintf(stderr, "socket() failed\n");
		return 1;
	}
	//first IP
	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	if (inet_aton(argv[1], &si_other.sin_addr)==0) {
		fprintf(stderr, "inet_aton() failed\n");
		return 1;
	}
	//second IP
	memset((char *) &si_other2, 0, sizeof(si_other2));
	si_other2.sin_family = AF_INET;
	si_other2.sin_port = htons(PORT);
	if (inet_aton(argv[2], &si_other2.sin_addr)==0) {
		fprintf(stderr, "inet_aton() failed\n");
		return 1;
	}
	//set cameras
	TDepthCamera mainCam;
	createPrimaryCamera(&mainCam, 0);
	//get calibration values acquired by calibration program.
	FILE* pFile = NULL;
	pFile = fopen("calibrationValuesOne.cal", "r");
	if(pFile == NULL){
		puts("Could not get calibration data.");
	}else{
		fread(&minZ, sizeof(int), 1, pFile);
		fread(&maxZ, sizeof(int), 1, pFile);
	}
	fclose(pFile);
	TVecList mainList;
	unsigned int timestamp;
	contLoop = 1;
	//show current calibration values.
	printf("Current calibration values:\nCeiling: %d, Floor: %d\n", maxZ, minZ);
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
		//acquire data for main Kinect & process data
		if(updateCamera(&mainCam, &timestamp)){
            printf("Could not update feed for device 0.");
            return EXIT_FAILURE;
		}
		if(detectDrone(mainCam.data, &mainList, &vec3DDistance)){
            printf("Could not process data for for device 0.");
            return EXIT_FAILURE;
		}
		//display list
		system("clear");
		puts("Press Enter to exit.\n\n---------------\nLIST:");
		displayVecList(&mainList);
		//send position to the given IP address
		TVec4D* maxVect = maxPointList(&mainList);
		if(maxVect != NULL){
            writePacket(buf, 'k', maxVect->x, maxVect->y, maxVect->z);
			if (sendto(s, buf, BUFLEN, 0, &si_other, slen)==-1){
				fprintf(stderr, "sendto() failed\n");
				return 1;
			}
			if (sendto(s, buf, BUFLEN, 0, &si_other2, slen)==-1){
				fprintf(stderr, "sendto() failed\n");
				return 1;
			}
		}
	}
	//close socket
	close(s);
	//free all data
	freeCamera(&mainCam);
	//stop kinects
	freenect_sync_stop();
	//stop pthread
	pthread_exit(NULL);
	return EXIT_SUCCESS;
}

/**
 * Writes data to a packet which will then be sent via the UDP socket.
 * A 8 bit checksum is written at the end of the packet.
 *
 * @param Pointer to the packet. The packet must be at least 8 bytes long.
 * @param Type of data transmitted.
 * @param First variable to transmit.
 * @param Second variable to transmit.
 * @param Third variable to transmit.
 */
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

/**
 * Function executed in a thread to asynchronously end the infinite loop.
 *
 * @param Pointer to the thread arguments.
 */
void *readAsync(void *threadid)
{
   getchar();
   contLoop = 0;
   pthread_exit(NULL);
}
