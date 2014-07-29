#ifndef KINECT_DETECTION_UTIL_H
#define KINECT_DETECTION_UTIL_H

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

///global variables
extern int nbIterations;
extern int minDepth;
extern int maxDepth;
extern int minZ;
extern int maxZ;

///prototypes
//vector functions
void vec4DFromDepth(TVec4D* vec, float xs, float ys, float depth);
float vec2DDistance(const TVec4D* v1, const TVec4D* v2);
float vec3DDistance(const TVec4D* v1, const TVec4D* v2);
float vecHeightDifference(const TVec4D* v1, const TVec4D* v2);
//matrix functions
TMatrix4D* matrix4DIdentity();
TMatrix4D* matrix4DTranslationRotationZ(float x, float y, float z, float angle);
void transformVec4D(TVec4D* vec, const TMatrix4D* matr);
void matrix4DGetFromVectors(TMatrix4D* m, const TVec4D* vectors);
float matrix4DCofactor(const TMatrix4D* m, int x, int y);
void matrix4DMultiply(TMatrix4D* result, const TMatrix4D* m1, const TMatrix4D* m2);
int matrix4DInvert(TMatrix4D* invert, const TMatrix4D* m);
//camera functions
void createPrimaryCamera(TDepthCamera* pCamera, int id);
void createSecondaryCamera(TDepthCamera* pCamera, int id, float x, float y, float z, float angle);
int updateCamera(TDepthCamera* pCamera, unsigned int* timestamp);
void freeCamera(TDepthCamera* pCamera);
//vector list functions
void resetVecList(TVecList* list);
int addVecToList(TVecList* list, const TVec4D* vec, int weight, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
int getMaxVectorFromList(TVec4D* v, TVecList* list);
TVec4D* maxPointList(TVecList* list);
//data analysis functions
int detectDrone(short* data, TVecList* list, float vecDistance(const TVec4D*, const TVec4D*));
int fusePointList(TVecList* mainList, const TVecList* secList, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
int __simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
int simplifyPointList(TVecList* list, float tolerance, float vecDistance(const TVec4D*, const TVec4D*));
//display functions
void displayVec4(const TVec4D* v);
void displayMatrix4(const TMatrix4D* m);
void displayVecList(const TVecList* list);

#endif
