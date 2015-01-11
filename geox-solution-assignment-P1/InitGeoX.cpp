//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "InitGeoX.h"
//---------------------------------------------------------------------------
#include "Camera.h"
#include "ViewFrustum.h"
#include "CameraController.h"
#include "ClassCreator.h"
#include "ClassEditor.h"
#include "ExaminerCameraController.h"
#include "Persistent.h"
#include "PropertyBrowser.h"
#include "PropertyEditor.h"
#include "WalkthroughCameraController.h"
#include "StreamClassTable.h"
#include "LegacyClassNameTranslation.h"

#include "BooleanPropertyEditor.h"
#include "ObjectPropertyEditor.h"
#include "FixedArrayPropertyEditor.h"
#include "ClassTypeClassProperty.h"
#include "ClassTypeClassPropertyEditorWidget.h"

#include "Experiment.h"
#include "ExampleExperiment.h"
#include "ExampleExperiment2DGraphics.h"
#include "ExampleExperiment3DGraphics.h"
#include "ExampleExperimentMarchingCubes.h"
#include "MeshImporterExperiment.h"
#include "MyMeshExperiment.h"
#include "SimpleGLMeshMaterial.h"

#include "StructureDescriptor.h"
#include "DynamicArrayOfStructures.h"
#include "PointCloud.h"
#include "TriangleMesh.h"

// ...
// include your experiments here
// ...

//---------------------------------------------------------------------------

void init() {

	Object::init(NULL);
	ClassCreator::init(Object::getClass());
	PropertyEditor::init(Object::getClass());
	BooleanPropertyEditor::init(PropertyEditor::getClass());
	ObjectPropertyEditor::init(PropertyEditor::getClass());
	FixedArrayPropertyEditor::init(PropertyEditor::getClass());
	ClassEditor::init(Object::getClass());
	PropertyBrowser::init(ClassEditor::getClass());
	Persistent::init(Object::getClass());
	StreamClassTable::init(Persistent::getClass());
	Camera::init(Persistent::getClass());
	ViewFrustum::init(Persistent::getClass());
	CameraController::init(Persistent::getClass());
	ExaminerCameraController::init(CameraController::getClass());
	WalkthroughCameraController::init(CameraController::getClass());

	Experiment::init(Persistent::getClass());
	ExampleExperiment::init(Experiment::getClass());
	ExampleExperiment2DGraphics::init(Experiment::getClass());
	ExampleExperiment3DGraphics::init(Experiment::getClass());
	ExampleExperimentMarchingCubes::init(Experiment::getClass());
	MeshImporterExperiment::init(Experiment::getClass());
	MyMeshExperiment::init(Experiment::getClass());
	SimpleGLMeshMaterial::init(Persistent::getClass());

   StructureDescriptor::init(Persistent::getClass());
   DynamicArrayOfStructures::init(Persistent::getClass());
   PointCloud::init(Persistent::getClass());
   TriangleMesh::init(PointCloud::getClass());

   // ...
   // register your experiments here
   // ...
}

void shutdown() {
	Object::shutdown();
	ClassEditor::shutdown();
	Persistent::shutdown();
	StreamClassTable::shutdown();
	Camera::shutdown();
	ViewFrustum::shutdown();
	CameraController::shutdown();
	ExaminerCameraController::shutdown();
	WalkthroughCameraController::shutdown();
	BooleanPropertyEditor::shutdown();
	ObjectPropertyEditor::shutdown();
	FixedArrayPropertyEditor::shutdown();
	PropertyBrowser::shutdown();
	
	Experiment::shutdown();
	ExampleExperiment::shutdown();
	ExampleExperiment2DGraphics::shutdown();
	ExampleExperiment3DGraphics::shutdown();
	ExampleExperimentMarchingCubes::shutdown();
	MeshImporterExperiment::shutdown();
	MyMeshExperiment::shutdown();
	SimpleGLMeshMaterial::shutdown();

	StructureDescriptor::shutdown();
	DynamicArrayOfStructures::shutdown();
	PointCloud::shutdown();
	TriangleMesh::shutdown();
	
	// ...
   // unregister your experiments here
   // ...
}
