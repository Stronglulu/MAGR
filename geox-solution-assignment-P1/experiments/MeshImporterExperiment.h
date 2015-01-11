//---------------------------------------------------------------------------
#ifndef MeshImporterExperimentH
#define MeshImporterExperimentH
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
class MeshImporterExperiment : public Experiment, public RenderableObject {
   GEOX_CLASS(MeshImporterExperiment)
	private:
		BasicGLViewer3D* viewer;
		TriangleMesh *mesh;
		SimpleGLMeshMaterial *renderer;
	public:                                       

		MeshImporterExperiment();

		virtual void renderGL();
		void importMesh();
		virtual QWidget *createViewer();

		~MeshImporterExperiment();
};


#endif                                         
