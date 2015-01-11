//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "TriangleMesh.h"
//---------------------------------------------------------------------------
#include "HashTables.hpp"
#include "LinearAlgebra.hpp"
#include "DynamicArrayOfStructures.hpp"
#include <set>
#include <queue>
#include "Object.h"
#include "ObjectClassProperty.h"
#include "StructureDescriptor.hpp"
//---------------------------------------------------------------------------


IMPLEMENT_GEOX_CLASS(TriangleMesh, 0) {
	BEGIN_CLASS_INIT( TriangleMesh );
	ADD_OBJECT_PROP( triangles, 0, DynamicArrayOfStructures::getClass(), true );
}



// ----


TriangleMesh::TriangleMesh()
{
   triangles = new DynamicArrayOfStructures();
   StructureDescriptor vdp;
   vdp.pushAttrib(mSAD("position", 3, SAD::DATA_FORMAT_FLOAT32));
   StructureDescriptor vdt;
   vdt.pushAttrib(mSAD("index", 3, SAD::DATA_FORMAT_INT32));
   clearAndSetup(&vdp, 0, &vdt, 0);
}

void TriangleMesh::clearAndSetup(const StructureDescriptor *vertDescr, mpcard numPoints,
                                                   const StructureDescriptor *triangleDescr, mpcard numTriangles)
{
   PointCloud::clearAndSetup(vertDescr, numPoints);
   triangles->clearAndSetup(1, numTriangles, triangleDescr);
}                                            

void TriangleMesh::clearAndSetup( mpcard numPoints, mpcard numTriangles, bool hasNormal/*=true*/, bool hasColor /*= true*/, bool hasFlags /*= true*/ )
{
   PointCloud::clearAndSetup(numPoints,true,hasNormal,hasColor,hasFlags);
   StructureDescriptor vdTri;
   vdTri.pushAttrib(SAD("index",3,SAD::DATA_FORMAT_INT32));
   triangles->clearAndSetup(1,numTriangles,&vdTri);
}


void TriangleMesh::assign(const Object* obj, CopyContextt *context) {
   PointCloud::assign(obj, context);
   const TriangleMesh *other = dynamic_cast<const TriangleMesh*>(obj);
   pAssert(other);
   if (other != this) {
      triangles->assign(other->triangles);
   }
}

void TriangleMesh::clear() {
   PointCloud::clear();
   triangles->clear();
}

void TriangleMesh::setTriangles(DynamicArrayOfStructures *psTriangles) {
   if (triangles != psTriangles) {
      delete triangles;
      triangles = psTriangles;
   }
}

TriangleMesh::~TriangleMesh() {
   delete triangles;
}







