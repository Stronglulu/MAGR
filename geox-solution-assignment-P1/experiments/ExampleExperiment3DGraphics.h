//---------------------------------------------------------------------------
#ifndef ExampleExperiment3DGraphicsH
#define ExampleExperiment3DGraphicsH
//---------------------------------------------------------------------------
#include "Experiment.h"
#include "LinearAlgebra.h"
#include "GLGeometryViewer3D.h"
//---------------------------------------------------------------------------

///
/// This is an example experiment.
///
/// The code is meant to demonstrate how
///  to use the GeoX framework
///
class ExampleExperiment3DGraphics : public Experiment {
   GEOX_CLASS(ExampleExperiment3DGraphics)
	private:
		GLGeometryViewer3D* viewer;
	public:                                       
		ExampleExperiment3DGraphics();
		~ExampleExperiment3DGraphics();
		void addTriangles();
		virtual QWidget *createViewer();
};


#endif                                         
