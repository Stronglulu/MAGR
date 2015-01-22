//---------------------------------------------------------------------------
#ifndef MyMeshExperimentH
#define MyMeshExperimentH
//---------------------------------------------------------------------------
#include "Experiment.h"
#include "LinearAlgebra.h"
#include "BasicGLViewer3D.h"
#include "BasicGLViewer3D.h"
//---------------------------------------------------------------------------


class TriangleMesh;
class SimpleGLMeshMaterial;

///
/// This is an example experiment that demonstrates importing and rendering triangle meshes.
///
class MyMeshExperiment: public Experiment, public RenderableObject {
	GEOX_CLASS(MyMeshExperiment)
private:
	BasicGLViewer3D* viewer;
	TriangleMesh *mesh;
	SimpleGLMeshMaterial *renderer;
	ExaminerCameraController* controller;

	// Image 
	int 	size = 100;
	tuple<Vector3f, Vector3f> rays[101][101];
	Vector3f colours[101][101];
	Vector3f reflectRays[101][101];
	float minDistances[101][101];
	const static int rayContainerSize = 101;	// Stores rays for jitter
	
	// Jitter
	const static int gridSize = 10;						// <-- SHOULD ONLY BE UNEVEN
	const static int numSubRays = gridSize * gridSize;
	const static int gridSpacer = (gridSize - 1) / 2;
	bool randomIsOn = false;
	Vector3f gridRays[numSubRays];
	tuple<Vector3f, Vector3f> raysGridded[rayContainerSize][rayContainerSize][numSubRays + 1]; //[Heighth][Width][numRays]. 1-gridSq reserved for random, last for center ray.
	bool checkShadow2(tuple<Vector3f, Vector3f>, float, Vector3f, card32);


	int checkerSize = 3;
	Vector3f lightPos = makeVector3f(0, 100, 0);
	bool jitter = true;
	bool pathTracing = true;
	int maxDepth = 4;
	float absorbtionP = 0.9;
	float lightIntensity = 10000;
	float normalizeColor;
	float normalizeFactor;

	

public:

	MyMeshExperiment();

	virtual void renderGL();
	void importMesh();
	virtual QWidget *createViewer();

	void getViewerInfo();
	void getRays();
	void shootRays();
	bool checkShadow(tuple<Vector3f, Vector3f>, float,Vector3f);
	Vector3f MyMeshExperiment::calculateShadow(tuple<Vector3f, Vector3f>, float, float);
	float getSoftShadow(tuple<Vector3f, Vector3f>, float, Vector3f);
	int mod(float, int);
	Vector3f getPlaneColor(Vector3f);
	float getLocalWeight(Vector3f, Vector3f[]);
	Vector3f getAverageSubPixels(Vector3f[], int);
	Vector3f getTotalSubPixels(Vector3f rays[], int length);
	//float getPathTracing(tuple<Vector3f, Vector3f>, int);
	Vector3f getPathTracing(tuple<Vector3f, Vector3f>, int);
	Vector3f getColour(Vector3f);
	bool MyMeshExperiment::checkRecursionShadow(Vector3f hit, Vector3f lightPos);

	//int32 gridSize;
	Vector3f incomingRay;                           // <---
	Vector3f vertex1;
	Vector3f vertex2;
	Vector3f vertex3;
	bool softShadows = false;
	bool perfectReflection = true;
	bool checkeredPlane = false;

	//Matrix3f triangleRefl;                           // <--- declare the parameters you need (will be registered in *.cpp file)

	void calculateDot();
	void calculateMatrixNormal();

	void calculateSurfaceNormal(Vector3f triangle[3], Vector3f &normal);
	void getOutgoingReflection(Vector3f incomingRay, Vector3f triangle[3], Vector3f &outgoingRay);
	void saveImage();


	~MyMeshExperiment();
};


#endif                                         
