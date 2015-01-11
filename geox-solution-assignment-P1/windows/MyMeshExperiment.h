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
	tuple<Vector3f, Vector3f> rays[101][101];
	Vector3f colours[101][101];
	int 	size = 100;
public:

	MyMeshExperiment();

	virtual void renderGL();
	void importMesh();
	virtual QWidget *createViewer();

	void getViewerInfo();
	void getRays();
	void shootRays();
	bool checkShadow(tuple<Vector3f, Vector3f>, float);

	int32 gridSize;
	Vector3f incomingRay;                           // <---
	Vector3f vertex1;
	Vector3f vertex2;
	Vector3f vertex3;
	//Matrix3f triangleRefl;                           // <--- declare the parameters you need (will be registered in *.cpp file)

	void calculateDot();
	void calculateMatrixNormal();

	void calculateSurfaceNormal(Vector3f triangle[3], Vector3f &normal);
	void getOutgoingReflection(Vector3f incomingRay, Vector3f triangle[3], Vector3f &outgoingRay);
	void saveImage();


	~MyMeshExperiment();
};


#endif                                         
