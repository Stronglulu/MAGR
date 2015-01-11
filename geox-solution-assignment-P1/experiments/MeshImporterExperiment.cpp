//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "MeshImporterExperiment.h"
//---------------------------------------------------------------------------
#include "Properties.h"
#include "GLGeometryViewer3D.h"
#include "GeoXOutput.h"   
#include "MeshImporterSMFOBJ.h"
#include "SimpleGLMeshMaterial.h"
#include "TriangleMesh.h"
#include "ObjectViewsTable.h"
//---------------------------------------------------------------------------

IMPLEMENT_GEOX_CLASS( MeshImporterExperiment ,0)
{
   BEGIN_CLASS_INIT( MeshImporterExperiment );
	ADD_OBJECT_PROP(mesh, 0, TriangleMesh::getClass(), true)
	ADD_OBJECT_PROP(renderer, 0, SimpleGLMeshMaterial::getClass(), true)
   ADD_NOARGS_METHOD(MeshImporterExperiment::importMesh)
}

QWidget *MeshImporterExperiment::createViewer() {
   viewer = new BasicGLViewer3D();
	viewer->addRenderCallback(this);
   return viewer;
}

MeshImporterExperiment::MeshImporterExperiment() 
{
	viewer = NULL;
	mesh = NULL;
	renderer = new SimpleGLMeshMaterial();
}

MeshImporterExperiment::~MeshImporterExperiment()
{
	delete renderer;
}



void MeshImporterExperiment::importMesh() 
{
	
	QString filename = QFileDialog::getOpenFileName(viewer, "Open OBJ File...", QString(), "3D Objects (*.obj *.smf)", 0, 0 );
	if (filename != "") {
		delete mesh;
		mesh = NULL;
		MeshImporterSMFOBJ importer;
		importer.performImport(qString2STLString(filename));
		mesh = importer.createTriangleMesh();
	}

	// display changes
	ObjectViewsTable::update(this);
	viewer->refresh();
}

void MeshImporterExperiment::renderGL()
{
	if (renderer && mesh) {
		renderer->draw(mesh);
	}
}

