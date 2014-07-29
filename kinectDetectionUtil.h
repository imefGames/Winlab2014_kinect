#pragma once

#define MAXVECTORS 16

/// Structure for 4-dimension vectors.
typedef struct{
	float x, y, z, w;
}TVec4D;

/// Structure for 4x4 matrices.
typedef struct{
	float m[16];
}TMatrix4D;

/// Structure representing a Kinect.
/// The id corresponds to the id of the Kinect.
/// The base is a transformation to apply on vectors if necessary.
/// The data contains the depth map if the camera.
/// The depth is given in millimetres.
typedef struct{
	int id;
	TMatrix4D* base;
	short* data;
}TDepthCamera;

/// Structure containing a list of vectors.
/// Each vector has an associated weight indicating its importance.
typedef struct{
	TVec4D vector[MAXVECTORS];
	short weight[MAXVECTORS];
	int n;
}TVecList;

///global variables
extern int nbIterations;
extern int minDepth;
extern int maxDepth;
extern int minZ;
extern int maxZ;


/**
 * Converts a given depth pixel into 3D coordinates.
 *
 * @param Pointer to the vector
 * @param x coordinate on the depth map
 * @param y coordinate on the depth map
 * @param Depth value on the depth map
 */
void vec4DFromDepth(TVec4D* vec, float xs, float ys, float depth);

/**
 * Returns the distance between 2 vectors only taking into account the x and y coordinates.
 *
 * @param Pointer to the first vector
 * @param Pointer to the second vector
 */
float vec2DDistance(const TVec4D* v1, const TVec4D* v2);

/**
 * Returns the distance between 2 vectors only taking into account the x, y, and z coordinates.
 *
 * @param Pointer to the first vector
 * @param Pointer to the second vector
 */
float vec3DDistance(const TVec4D* v1, const TVec4D* v2);

/**
 * Returns the distance between 2 vectors only taking into account the z coordinates.
 * The result is the absolute value of the difference between both z values.
 *
 * @param Pointer to the first vector
 * @param Pointer to the second vector
 */
float vecHeightDifference(const TVec4D* v1, const TVec4D* v2);

/**
 * Returns a new identity matrix.
 */
TMatrix4D* matrix4DIdentity();

/**
 * Returns a new transformation matrix.
 * The transformation performs a rotation around the Z axis and a translation.
 *
 * @param x value of translation
 * @param y value of translation
 * @param z value of translation
 * @param Angle of rotation in radians.
 */
TMatrix4D* matrix4DTranslationRotationZ(float x, float y, float z, float angle);

/**
 * Applies a transformation matrix to a vector.
 *
 * @param Pointer to the vector
 * @param Pointer to the matrix
 */
void transformVec4D(TVec4D* vec, const TMatrix4D* matr);

/**
 * Creates a 4x4 matrix from a list of 4 vectors.
 *
 * @param Pointer to the matrix
 * @param Pointer to the vector list
 */
void matrix4DGetFromVectors(TMatrix4D* m, const TVec4D* vectors);

/**
 * Multiplies 2 matrices.
 *
 * @param Pointer to the matrix which will contain the result of the multiplication
 * @param Pointer to the first matrix
 * @param Pointer to the second matrix
 */
void matrix4DMultiply(TMatrix4D* result, const TMatrix4D* m1, const TMatrix4D* m2);

/**
 * Returns the cofactor of a matrix at given (x,y) coordinates.
 *
 * @param Pointer to the matrix
 * @param x coordinate of the cofactor
 * @param y coordinate of the cofactor
 */
float matrix4DCofactor(const TMatrix4D* m, int x, int y);

/**
 * Inverts a matrix.
 * Returns 0 if inversion is a success and 1 if inversion is not possible.
 *
 * @param Pointer to the inverted matrix
 * @param Pointer to the matrix to invert
 */
int matrix4DInvert(TMatrix4D* invert, const TMatrix4D* m);

/**
 * Creates a primary camera.
 * A primary camera has an identity matrix as a base.
 *
 * @param Pointer to the camera
 * @param ID of the camera
 */
void createPrimaryCamera(TDepthCamera* pCamera, int id);

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
void createSecondaryCamera(TDepthCamera* pCamera, int id, float x, float y, float z, float angle);

/**
 * Refreshes the depth map of a camera.
 *
 * @param Pointer to the camera
 * @param Pointer to the time stamp
 */
int updateCamera(TDepthCamera* pCamera, unsigned int* timestamp);

/**
 * Frees a camera.
 * Well actually, it only frees the base matrix if the camera.
 *
 * @param Pointer to the camera
 */
void freeCamera(TDepthCamera* pCamera);

/**
 * Empties a vector list and resets the weight of all vectors to 1.
 *
 * @param Pointer to the vector list
 */
void resetVecList(TVecList* list);

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
int addVecToList(TVecList* list, const TVec4D* vec, int weight, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));

/**
 * Copies the vector with the highest weight in a list.
 * Returns 1 if the list is empty and 0 if the operation is a success.
 *
 * @param Pointer to the vector
 * @param Pointer to the vector list
 */
int getMaxVectorFromList(TVec4D* v, TVecList* list);

/**
 * Returns the address of the vector with the highest weight.
 * Returns NULL if the list is empty.
 *
 * @param Pointer to the vector list
 */
TVec4D* maxPointList(TVecList* list);

/**
 * Processes a depth map to generate a list of vectors.
 * The number of iterations can be changed with the global variable nbIterations.
 * Returns 0 if the operation is a success and 1 in case of a failure.
 *
 * @param Pointer to the depth map
 * @param Pointer to the vector list
 * @param Function used to determine the distance between two vectors
 */
int detectDrone(short* data, TVecList* list, float vecDistance(const TVec4D*, const TVec4D*));

/**
 * Adds all the vectors of the second list to the first list.
 * If two vectors are close enough the are fused.
 *
 * @param Pointer to the first vector list
 * @param Pointer to the second vector list
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
int fusePointList(TVecList* mainList, const TVecList* secList, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));

/**
 * Fuses vectors within a same list if they are close enough.
 *
 * @param Pointer to the vector list
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
int __simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));

/**
 * Applies the function __simplifyPointList until no more fusions are possible.
 *
 * @param Pointer to the vector list
 * @param Tolerance for fusing two vectors
 * @param Function used to determine the distance between two vectors
 */
int simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));

/**
 * Displays the (x,y,z,w) coordinates of a vector.
 *
 * @param Pointer to the vector
 */
void displayVec4(const TVec4D* v);

/**
 * Displays a matrix.
 *
 * @param Pointer to the vector
 */
void displayMatrix4(const TMatrix4D* m);

/**
 * Displays all the vectors of a vector list.
 *
 * @param Pointer to the vector
 */
void displayVecList(const TVecList* list);