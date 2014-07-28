// Be sure to link with -lfreenect_sync -lm
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <libfreenect_sync.h>

#define MAXVECTORS 16

///types
typedef struct{
	float x, y, z, w;
}TVec4D;

typedef struct{
	float m[16];
}TMatrix4D;

typedef struct{
	int id;
	TMatrix4D* base;
	short* data;
}TDepthCamera;

typedef struct{
	TVec4D vector[MAXVECTORS];
	short weight[MAXVECTORS];
	int n;
}TVecList;

///prototypes
//vector functions
void vec4DFromDepth(TVec4D* vec, float xs, float ys, float depth);
float vec2DDistance(const TVec4D* v1, const TVec4D* v2);
float vec3DDistance(const TVec4D* v1, const TVec4D* v2);
float vecHeightDifference(const TVec4D* v1, const TVec4D* v2);
//matrix functions
TMatrix4D* matrix4DIdentity();
void matrix4DGetFromVectors(TMatrix4D* m, const TVec4D* vectors);
float matrix4DCofactor(const TMatrix4D* m, int x, int y);
void matrix4DMultiply(TMatrix4D* result, const TMatrix4D* m1, const TMatrix4D* m2);
int matrix4DInvert(TMatrix4D* invert, const TMatrix4D* m);
//camera functions
void createPrimaryCamera(TDepthCamera* pCamera, int id);
int updateCamera(TDepthCamera* pCamera, unsigned int* timestamp);
void freeCamera(TDepthCamera* pCamera);
//vector list functions
void resetVecList(TVecList* list);
int addVecToList(TVecList* list, const TVec4D* vec, int weight, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
int getMaxVectorFromList(TVec4D* v, TVecList* list);
//data analysis functions
int detectDrone(short* data, TVecList* list, float vecDistance(const TVec4D*, const TVec4D*));
int fusePointList(TVecList* mainList, const TVecList* secList, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
int __simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
int simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
//display functions
void displayVec4(const TVec4D* v);
void displayMatrix4(const TMatrix4D* m);
void displayVecList(const TVecList* list);

///global variables
int nbIterations = 200000;//90% => 245, 95% => 319, 99% => 489
int minDepth = 400;
int maxDepth = 6000;
int minZ = -1000;
int maxZ = 1000;

///functions
int main(int argc, char* argv[])
{
	//input parametres
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

//vector functions
void vec4DFromDepth(TVec4D* vec, float xs, float ys, float depth){
	//converting depth to (x, y, z)
	depth += 50;
	//vec->x = depth*tan((xs-320)*0.001554434);
	vec->x = depth*(xs-320)*0.00169673656;
	//vec->z = depth*tan((240-ys)*0.001563524);
	vec->z = depth*(240-ys)*0.00164129365;
	vec->y = depth;
	vec->w = 1;
}

float vec2DDistance(const TVec4D* v1, const TVec4D* v2){
	float dx, dy;
	dx = v1->x - v2->x;
	dy = v1->y - v2->y;
	return sqrt(dx*dx + dy*dy);
}

float vec3DDistance(const TVec4D* v1, const TVec4D* v2){
	float dx, dy, dz;
	dx = v1->x - v2->x;
	dy = v1->y - v2->y;
	dz = v1->z - v2->z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

float vecHeightDifference(const TVec4D* v1, const TVec4D* v2){
    return v1->z>=v2->z? v1->z-v2->z : v2->z-v1->z;
}

//matrix functions
TMatrix4D* matrix4DIdentity(){
	//Allocation of space for the matrix
	TMatrix4D* matr = NULL;
	matr = malloc(sizeof(TMatrix4D));
	if(matr == NULL){ return NULL; }
	//generating matrix
	int i, j;
	for(j=0; j<4; j++){
		for(i=0; i<4; i++){
			if(i==j){
				matr->m[j*4+i] = 1;
			}else{
				matr->m[j*4+i] = 0;
			}
		}
	}
	//returning matrix
	return matr;
}

void matrix4DGetFromVectors(TMatrix4D* m, const TVec4D* vectors){
    int i;
    for(i=0; i<4; i++){
        m->m[i] = vectors[i].x;
        m->m[i+4] = vectors[i].y;
        m->m[i+8] = vectors[i].z;
        m->m[i+12] = vectors[i].w;
    }
}

void matrix4DMultiply(TMatrix4D* result, const TMatrix4D* m1, const TMatrix4D* m2){
    int i, j, k;
    for(i=0; i<4; i++){
        for(j=0; j<4; j++){
            result->m[i+j*4] = m1->m[j*4] * m2->m[i];
            for(k=1; k<4; k++){
                result->m[i+j*4] += m1->m[k+j*4] * m2->m[i+k*4];
            }
        }
    }
}

float matrix4DCofactor(const TMatrix4D* m, int x, int y){
    float subM[9];
    int i, j, k=0, l=0;
    for(j=0; j<4; j++){
        k=0;
        if(j==y){ continue; }
        for(i=0; i<4; i++){
            if(i!=x){
                subM[k+3*l] = m->m[i+j*4];
                k++;
            }
        }
        l++;
    }
    return subM[0]*subM[4]*subM[8] + subM[1]*subM[5]*subM[6] + subM[2]*subM[3]*subM[7] - subM[2]*subM[4]*subM[6] - subM[1]*subM[3]*subM[8] - subM[0]*subM[5]*subM[7];
}

int matrix4DInvert(TMatrix4D* invert, const TMatrix4D* m){
    int i, j;
    float det = 0;
	for(i=0; i<4; i++){
		for(j=0;j<4; j++){
			int s;
			if((i+j)%2 == 0){ s=1; }else{ s=-1; }
			invert->m[j+4*i] = s*matrix4DCofactor(m, i, j);
		}
        det += m->m[i]*invert->m[4*i];
	}
	if(det == 0){
        return 1;
	}
	for(i=0; i<4; i++){
		for(j=0;j<4; j++){
			invert->m[i+4*j] /= det;
		}
	}
	return 0;
}

//camera functions
void createPrimaryCamera(TDepthCamera* pCamera, int id){
	pCamera->id = id;
	pCamera->data = NULL;
	pCamera->base = matrix4DIdentity();
}

int updateCamera(TDepthCamera* pCamera, unsigned int* timestamp){
	return freenect_sync_get_depth((void**)(&(pCamera->data)), timestamp, pCamera->id, FREENECT_DEPTH_REGISTERED);
}

void freeCamera(TDepthCamera* pCamera){
	free(pCamera->base);
}

//vector list functions
void resetVecList(TVecList* list){
	list->n = 0;
	int i;
	for(i=0; i<MAXVECTORS; i++){
        /*list->vector[i].x = 0;
        list->vector[i].y = 0;
        list->vector[i].z = 0;
        list->vector[i].w = 0;*/
		list->weight[i] = 1;
	}
}

int addVecToList(TVecList* list, const TVec4D* vec, int weight, float tolerance, float vecDistance(const TVec4D*, const TVec4D*)){
	//compare with all existing vectors
	int i, fuse = 0;
	for(i=0; i<list->n; i++){
		//if 2 vectors are close
		if(vecDistance(vec, &(list->vector[i])) < tolerance){
			list->vector[i].x = list->vector[i].x*list->weight[i] + weight*vec->x;
			list->vector[i].y = list->vector[i].y*list->weight[i] + weight*vec->y;
			list->vector[i].z = list->vector[i].z*list->weight[i] + weight*vec->z;
			list->weight[i] += weight;
			list->vector[i].x /= list->weight[i];
			list->vector[i].y /= list->weight[i];
			list->vector[i].z /= list->weight[i];
			fuse = 1;
			break;
		}
	}
	//if vector close to no other
	if(!fuse){
		//if there is room in the table
		if(list->n < MAXVECTORS){
			//add vector at the end of table
			list->vector[list->n].x = vec->x;
			list->vector[list->n].y = vec->y;
			list->vector[list->n].z = vec->z;
			list->vector[list->n].w = vec->w;
			list->n++;
		}else{
			//table full
			return -1;
		}
	}
	//no problem
	return fuse;
}

int getMaxVectorFromList(TVec4D* v, TVecList* list){
    if(list->n == 0){ return 1; }
    int i, max = 0;
    for(i=1; i<list->n; i++){
        if(list->weight[i] > list->weight[max]){ max = i; }
    }
    v->x = list->vector[max].x;
    v->y = list->vector[max].y;
    v->z = list->vector[max].z;
    v->w = list->vector[max].w;
    return 0;
}

//data analysis functions
int detectDrone(short* data, TVecList* list, float vecDistance(const TVec4D*, const TVec4D*)){
    //if data or list missing, error
    if(data == NULL || list == NULL){ return 1; }
    //reset vector list
    resetVecList(list);
    TVec4D tmpVector;
    int i;
    for(i=0; i<nbIterations; i++){
        //for each random pixel
        int pixelPos = rand()%(640*480);
        //if the depth at that pixel between min and max...
        if(data[pixelPos]>minDepth && data[pixelPos]<maxDepth){
            //convert to a vector
            vec4DFromDepth(&tmpVector, pixelPos%640, pixelPos/640, data[pixelPos]);
            //if the z componant is between a min and max...
            if(tmpVector.z > minZ && tmpVector.z < maxZ){
                //add vector to list
                addVecToList(list, &tmpVector, 1, 300, vecDistance);
            }
        }
    }
    //no problem
    return 0;
}

int fusePointList(TVecList* mainList, const TVecList* secList, float tolerance, float vecDistance(const TVec4D*, const TVec4D*)){
    int i, err, ret = 0;
    for(i=0; i<secList->n; i++){
        err = addVecToList(mainList, &(secList->vector[i]), secList->weight[i], tolerance, vecDistance);
        if(err == 1){
            ret = 1;
        }else if(err == -1){
            ret = -1;
            break;
        }
    }
    return ret;
}

int __simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*)){
    int i, j, ret = 0;
    for(i=0; i<list->n; i++){
        for(j=i+1; j<list->n; j++){
            //if 2 vectors in the list are close...
            if(vecDistance(&(list->vector[i]), &(list->vector[j])) < tolerance){
                //fuse both vectors
                list->vector[i].x = list->vector[i].x*list->weight[i] + list->vector[j].x*list->weight[j];
                list->vector[i].y = list->vector[i].y*list->weight[i] + list->vector[j].y*list->weight[j];
                list->vector[i].z = list->vector[i].z*list->weight[i] + list->vector[j].z*list->weight[j];
                list->weight[i] += list->weight[j];
                list->vector[i].x /= list->weight[i];
                list->vector[i].y /= list->weight[i];
                list->vector[i].z /= list->weight[i];
                //remove the second vector
                int k;
                for(k=j+1; k<list->n; k++){
                    //shift all following vectors
                    list->vector[k-1].x = list->vector[k].x;
                    list->vector[k-1].y = list->vector[k].y;
                    list->vector[k-1].z = list->vector[k].z;
                    list->vector[k-1].w = list->vector[k].w;
                    list->weight[k-1] = list->weight[k];
                }
                list->n--;
                //set return flag
                ret = 1;
                j--;
            }
        }
    }
    return ret;
}

int simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*)){
    int ret = 0;
    do{
        ret = __simplifyPointList(list, tolerance, vecDistance);
    }while(ret == 1);
}

//display functions
void displayVec4(const TVec4D* v){
	printf("x:%3.2f, y:%3.2f, z:%3.2f, w:%3.2f", v->x, v->y, v->z, v->w);
}

void displayMatrix4(const TMatrix4D* m){
    printf("[[\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]\n", m->m[0], m->m[1], m->m[2], m->m[3]);
    printf(" [\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]\n", m->m[4], m->m[5], m->m[6], m->m[7]);
    printf(" [\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]\n", m->m[8], m->m[9], m->m[10], m->m[11]);
    printf(" [\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]]\n", m->m[12], m->m[13], m->m[14], m->m[15]);
}

void displayVecList(const TVecList* list){
	int i;
	printf("Vectors:%d\n", list->n);
	for(i=0; i<list->n; i++){
		printf("Weight:%d, ", list->weight[i]);
		displayVec4(&(list->vector[i]));
		putchar('\n');
	}
}
