//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "PointCloud.h"
//---------------------------------------------------------------------------
#include "DynamicArrayOfStructures.h"
#include "DynamicArrayOfStructures.hpp"
#include "ObjectClassProperty.h"
#include "Tools.h"
#include "StructureDescriptor.hpp"
//---------------------------------------------------------------------------


IMPLEMENT_GEOX_CLASS(PointCloud, 0) {
	BEGIN_CLASS_INIT( PointCloud );
	ADD_OBJECT_PROP( ps, 0, DynamicArrayOfStructures::getClass(), true );
}


// ----



PointCloud::PointCloud() {
   ps = new DynamicArrayOfStructures();
}

void PointCloud::assign(const Object* obj, CopyContextt *context) {
   const PointCloud *pobj = dynamic_cast<const PointCloud*>(obj);
   pAssert(pobj != NULL);
   if (pobj->ps == NULL) {
      delete ps;
      ps = NULL;
   } else if (ps != pobj->ps) {
      delete ps;
      ps = (DynamicArrayOfStructures*)(pobj->ps)->copy(context);
   }
}

const StructureDescriptor *PointCloud::getDescr() const {
   return ps->getDescr();
}

card64 PointCloud::getNumPoints() const {
	if (ps == NULL) return 0;
	return ps->getNumEntries();
}


void PointCloud::clear() {
   ps->clear();
}


void PointCloud::clearAndSetup(const StructureDescriptor *vertDescr, mpcard height) {
   ps->clearAndSetup(1, height, vertDescr);
}


void PointCloud::clearAndSetup( mpcard numPoints, bool hasPosition/*=true*/, bool hasNormal/*=true*/, bool hasColor/*=true*/, bool hasFlags/*=true*/ )
{
	StructureDescriptor vd;
	if( hasPosition )
		vd.pushAttrib(SAD("position",3,SAD::DATA_FORMAT_FLOAT32));
	if( hasNormal )
		vd.pushAttrib(SAD("normal",3,SAD::DATA_FORMAT_FLOAT32));
	if( hasColor )
		vd.pushAttrib(SAD("color",3,SAD::DATA_FORMAT_FLOAT32));
	if( hasFlags )
		vd.pushAttrib(SAD("flags",1,SAD::DATA_FORMAT_INT32));
	clearAndSetup(&vd,numPoints);
}

Vector3f PointCloud::computeCenterOfMass() const
{
	Vector3f result = NULL_VECTOR3F;
	if (getNumPoints() < 1)
		return result;
	
	AAT POS = getAAT("position");
	for (mpcard i = 0; i < getNumPoints(); ++i)
		result += ps->get3f(i, POS);
	result /= (float32)getNumPoints();
	return result;	
}

PointCloud *PointCloud::createCopyWithNewPointSet(DynamicArrayOfStructures *ps) const {
   DynamicArrayOfStructures *pscopy = this->ps;
   ((PointCloud*)this)->ps = NULL;
   PointCloud *result = (PointCloud*)copy();
   ((PointCloud*)this)->ps = pscopy;
   result->ps = ps;
   return result;
}

PointCloud::~PointCloud() {
   delete ps;
}



