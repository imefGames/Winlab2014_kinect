#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <libfreenect_sync.h>
#include "kinectDetectionUtil.h"

///global variables
int nbIterations = 4000;
int minDepth = 400;
int maxDepth = 6000;
int minZ = -1000;
int maxZ = 1000;

/**
 * Converts a given depth pixel into 3D coordinates.
 *
 * @param Pointer to the vector
 * @param x coordinate on the depth map
 * @param y coordinate on the depth map
 * @param Depth value on the depth map
 */
void vec4DFromDepth(TVec4D* vec, float xs, float ys, float depth){
	//converting depth to (x, y, z)
	depth += 280;
	//vec->x = depth*tan((xs-320)*0.001554434);
	vec->x = depth*(xs-320)*0.00169673656;
	//vec->z = depth*tan((240-ys)*0.001563524);
	vec->z = depth*(240-ys)*0.00164129365;
	vec->y = depth;
	vec->w = 1;
}

/**
 * Returns the distance between 2 vectors only taking into account the x and y coordinates.
 *
 * @param Pointer to the first vector
 * @param Pointer to the second vector
 */
float vec2DDistance(const TVec4D* v1, const TVec4D* v2){
	float dx, dy;
	dx = v1->x - v2->x;
	dy = v1->y - v2->y;
	return sqrt(dx*dx + dy*dy);
}

/**
 * Returns the distance between 2 vectors only taking into account the x, y, and z coordinates.
 *
 * @param Pointer to the first vector
 * @param Pointer to the second vector
 */
float vec3DDistance(const TVec4D* v1, const TVec4D* v2){
	float dx, dy, dz;
	dx = v1->x - v2->x;
	dy = v1->y - v2->y;
	dz = v1->z - v2->z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * Returns the distance between 2 vectors only taking into account the z coordinates.
 * The result is the absolute value of the difference between both z values.
 *
 * @param Pointer to the first vector
 * @param Pointer to the second vector
 */
float vecHeightDifference(const TVec4D* v1, const TVec4D* v2){
    return v1->z>=v2->z? v1->z-v2->z : v2->z-v1->z;
}

/**
 * Returns a new identity matrix.
 */
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

/**
 * Returns a new transformation matrix.
 * The transformation performs a rotation around the Z axis and a translation.
 *
 * @param x value of translation
 * @param y value of translation
 * @param z value of translation
 * @param Angle of rotation in radians.
 */
TMatrix4D* matrix4DTranslationRotationZ(float x, float y, float z, float angle){
	//Allocation of space for the matrix
	TMatrix4D* matr = NULL;
	matr = malloc(sizeof(TMatrix4D));
	if(matr == NULL){ return NULL; }
	//generating matrix
	matr->m[0] = cos(angle);
	matr->m[4] = sin(angle);
	matr->m[8] = 0;
	matr->m[12] = 0;
	matr->m[1] = -matr->m[4];
	matr->m[5] = matr->m[0];
	matr->m[9] = 0;
	matr->m[13] = 0;
	matr->m[2] = 0;
	matr->m[6] = 0;
	matr->m[10] = 1;
	matr->m[14] = 0;
	matr->m[3] = x;
	matr->m[7] = y;
	matr->m[11] = z;
	matr->m[15] = 1;
	//returning matrix
	return matr;
}

/**
 * Applies a transformation matrix to a vector.
 *
 * @param Pointer to the vector
 * @param Pointer to the matrix
 */
void transformVec4D(TVec4D* vec, const TMatrix4D* matr){
	float nx, ny, nz, nw;
	nx = vec->x*matr->m[0] + vec->y*matr->m[1] + vec->z*matr->m[2] + vec->w*matr->m[3];
	ny = vec->x*matr->m[4] + vec->y*matr->m[5] + vec->z*matr->m[6] + vec->w*matr->m[7];
	nz = vec->x*matr->m[8] + vec->y*matr->m[9] + vec->z*matr->m[10] + vec->w*matr->m[11];
	nw = vec->x*matr->m[12] + vec->y*matr->m[13] + vec->z*matr->m[14] + vec->w*matr->m[15];
	vec->x = nx;
	vec->y = ny;
	vec->z = nz;
	vec->w = nw;
}

/**
 * Creates a 4x4 matrix from a list of 4 vectors.
 *
 * @param Pointer to the matrix
 * @param Pointer to the vector list
 */
void matrix4DGetFromVectors(TMatrix4D* m, const TVec4D* vectors){
    int i;
    for(i=0; i<4; i++){
        m->m[i] = vectors[i].x;
        m->m[i+4] = vectors[i].y;
        m->m[i+8] = vectors[i].z;
        m->m[i+12] = vectors[i].w;
    }
}

/**
 * Multiplies 2 matrices.
 *
 * @param Pointer to the matrix which will contain the result of the multiplication
 * @param Pointer to the first matrix
 * @param Pointer to the second matrix
 */
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

/**
 * Returns the cofactor of a matrix at given (x,y) coordinates.
 *
 * @param Pointer to the matrix
 * @param x coordinate of the cofactor
 * @param y coordinate of the cofactor
 */
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

/**
 * Inverts a matrix.
 * Returns 0 if inversion is a success and 1 if inversion is not possible.
 *
 * @param Pointer to the inverted matrix
 * @param Pointer to the matrix to invert
 */
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

/**
 * Creates a primary camera.
 * A primary camera has an identity matrix as a base.
 *
 * @param Pointer to the camera
 * @param ID of the camera
 */
void createPrimaryCamera(TDepthCamera* pCamera, int id){
	pCamera->id = id;
	pCamera->base = matrix4DIdentity();
}

/**
 * Creates a secondary camera.
 * A secondary camera has a transformation matrix as a base.
 *
 * @param Pointer to the camera
 * @param ID of the camera
 * @param x value of translation
 * @param y value of translation
 * @param z value of translation
 * @param Angle of rotation in radians.
 */
void createSecondaryCamera(TDepthCamera* pCamera, int id, float x, float y, float z, float angle){
	pCamera->id = id;
	pCamera->base = matrix4DTranslationRotationZ(x, y, z, angle);
}

/**
 * Refreshes the depth map of a camera.
 *
 * @param Pointer to the camera
 * @param Pointer to the time stamp
 */
int updateCamera(TDepthCamera* pCamera, unsigned int* timestamp){
	return freenect_sync_get_depth((void**)(&(pCamera->data)), timestamp, pCamera->id, FREENECT_DEPTH_REGISTERED);
}

/**
 * Frees a camera.
 * Well actually, it only frees the base matrix if the camera.
 *
 * @param Pointer to the camera
 */
void freeCamera(TDepthCamera* pCamera){
	free(pCamera->base);
}

/**
 * Empties a vector list and resets the weight of all vectors to 1.
 *
 * @param Pointer to the vector list
 */
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

/**
 * Adds a vector to a list if there is enough space.
 * If the vector is close enough to another in the list, both vectors will be fused instead.
 *
 * @param Pointer to the vector list
 * @param Pointer to the vector
 * @param Weight of the vector
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
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

/**
 * Copies the vector with the highest weight in a list.
 * Returns 1 if the list is empty and 0 if the operation is a success.
 *
 * @param Pointer to the vector
 * @param Pointer to the vector list
 */
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

/**
 * Returns the address of the vector with the highest weight.
 * Returns NULL if the list is empty.
 *
 * @param Pointer to the vector list
 */
TVec4D* maxPointList(TVecList* list){
    int maxId = 0, i;
    if(list->n==0){
        return NULL;
    }
    for(i=0; i<list->n; i++){
        if(list->weight[i] > list->weight[maxId]){
            maxId = i;
        }
    }
    return &(list->vector[maxId]);
}

/**
 * Processes a depth map to generate a list of vectors.
 * The number of iterations can be changed with the global variable nbIterations.
 * Returns 0 if the operation is a success and 1 in case of a failure.
 *
 * @param Pointer to the depth map
 * @param Pointer to the vector list
 * @param Function used to determine the distance between two vectors
 */
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
            //if the z component is between a min and max...
            if(tmpVector.z > minZ && tmpVector.z < maxZ){
                //add vector to list
                addVecToList(list, &tmpVector, 1, 300, vecDistance);
            }
        }
    }
    //no problem
    return 0;
}

/**
 * Adds all the vectors of the second list to the first list.
 * If two vectors are close enough the are fused.
 *
 * @param Pointer to the first vector list
 * @param Pointer to the second vector list
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
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

/**
 * Fuses vectors within a same list if they are close enough.
 *
 * @param Pointer to the vector list
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
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

/**
 * Applies the function __simplifyPointList until no more fusions are possible.
 *
 * @param Pointer to the vector list
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
int simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*)){
    int ret = 0;
    do{
        ret = __simplifyPointList(list, tolerance, vecDistance);
    }while(ret == 1);
}

/**
 * Displays the (x,y,z,w) coordinates of a vector.
 *
 * @param Pointer to the vector
 */
void displayVec4(const TVec4D* v){
	printf("x:%3.2f, y:%3.2f, z:%3.2f, w:%3.2f", v->x, v->y, v->z, v->w);
}

/**
 * Displays a matrix.
 *
 * @param Pointer to the vector
 */
void displayMatrix4(const TMatrix4D* m){
    printf("[[\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]\n", m->m[0], m->m[1], m->m[2], m->m[3]);
    printf(" [\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]\n", m->m[4], m->m[5], m->m[6], m->m[7]);
    printf(" [\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]\n", m->m[8], m->m[9], m->m[10], m->m[11]);
    printf(" [\t%3.5f\t%3.5f\t%3.5f\t%3.5f\t]]\n", m->m[12], m->m[13], m->m[14], m->m[15]);
}

/**
 * Displays all the vectors of a vector list.
 *
 * @param Pointer to the vector
 */
void displayVecList(const TVecList* list){
	int i;
	//displays the number of vectors
	printf("Vectors:%d\n", list->n);
	//displays each vector
	for(i=0; i<list->n; i++){
		printf("Weight:%d, ", list->weight[i]);
		displayVec4(&(list->vector[i]));
		putchar('\n');
	}
}
