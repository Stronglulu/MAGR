//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "DynamicArrayOfStructures.h"
#include "DynamicArrayOfStructures.hpp"
//---------------------------------------------------------------------------
#include "BoundingBox.hpp"
#include "LinearAlgebra.hpp"
#include "Tools.h"
#include "Exceptions.h"
#include "NumericalClassProperty.h"
#include "StringClassProperty.h"
#include "GeoXOutput.h"
//---------------------------------------------------------------------------
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------



IMPLEMENT_GEOX_CLASS(DynamicArrayOfStructures, 0) {
   BEGIN_CLASS_INIT(DynamicArrayOfStructures)
   ADD_MAX_PLATFORM_CARD_PROP( width, 0 );
   //ADD_MAX_PLATFORM_CARD_PROP( height, 0 );
   ADD_STRING_PROP_METHOD( description, 0, &getDescrString, NULL );
	( (MetaClass *)getClass())->actProperty_setEditReadonly( true );
		SET_EDIT_READONLY( true );
		SET_STREAM_PROPERTY( false );
	//ADD_MAX_PLATFORM_CARD_PROP( allocatedHeight, 0 );
		//SET_USER_EDIT( false );
}

// ----

string DynamicArrayOfStructures::getDescrString() const {
   return vertDescr->getDescription();
}

DynamicArrayOfStructures::DynamicArrayOfStructures() {
   vertDescr = new StructureDescriptor();
   data = NULL;
   width = 0;
   height = 0;
   numEntries = 0;
   elementSize = 0;
   allocatedHeight=0;
   boundingBoxValid = false;
}


static inline int32 simpleLog2(card32 i) {
   int32 result = -1;
   while (i) {
      i >>= 1;
      result++;
   }
   return result;
}


void DynamicArrayOfStructures::assign(const Object* obj, CopyContextt *context) {
   const DynamicArrayOfStructures *other = dynamic_cast<const DynamicArrayOfStructures*>(obj);
   pAssert(other != NULL);
   free(data);
   delete vertDescr;
   *this = *other;

   const mpcard numBytes = other->elementSize * other->numEntries;
   this->data = (char*)malloc(numBytes);
   if (this->data == NULL && numBytes > 0) throw PException("PointSet::assign() - cannot allocate memory.");
   memcpy(this->data, other->data, numBytes);
   this->allocatedHeight = height;

   this->vertDescr = (StructureDescriptor*)(other->vertDescr->copy());
   this->boundingBoxValid = false;
   forget(context);
}

void DynamicArrayOfStructures::write(OutputObjectStream *s) const {
   s->writeObject(vertDescr);
   // write data
   s->writeMaxPlatformCard(width);
   s->writeMaxPlatformCard(height);
   pAssert(numEntries == width * height);
   s->writeMaxPlatformCard(elementSize); // for checking only!
   s->writeBuffer(data, numEntries*elementSize);
}

void DynamicArrayOfStructures::read(InputObjectStream *s) {
   delete vertDescr;
   s->readObject((Persistent*&)vertDescr);
   if(vertDescr)
      elementSize = vertDescr->getSize();
   else
      elementSize = 0;

   // retrieve extends
   bool version32bit = (s->getClassVersion(DynamicArrayOfStructures::getClass()) <= MetaClass::FIRST_VERSION);
   if (version32bit) {
      card32 temp;
      s->read(temp);
      width = temp;
   } else {
      s->readMaxPlatformCard(width);
   }
   if (version32bit) {
      card32 temp;
      s->read(temp);
      height = temp;
   } else {
      s->readMaxPlatformCard(height);
   }
   numEntries = width*height;
   mpcard elementSizeTest;

   // only to check!
   if (version32bit) {
      card32 temp;
      s->read(temp);
      elementSizeTest = temp;
   } else {
      s->readMaxPlatformCard(elementSizeTest);
   }
   pAssert(elementSize == elementSizeTest);

   // read data
   free(data);
   data = (char*)malloc(numEntries * elementSize);
   if (data == NULL && numEntries * elementSize > 0) throw PException("PointSet::read() - cannot allocate memory.");
   s->readBuffer(data, numEntries*elementSize);

   this->allocatedHeight = height;
   this->boundingBoxValid = false;
}

void DynamicArrayOfStructures::clear() {
   width = 0;
   height = 0;
   numEntries = 0;
   free(data);
   data = NULL;
   allocatedHeight = height;
   boundingBoxValid = false;
}

void DynamicArrayOfStructures::clearAndSetup(mpcard width, mpcard height, const StructureDescriptor *vertDescr) {
   this->width = width;
   this->height = height;
   this->numEntries = width * height;
   if (this->vertDescr != vertDescr) {
      delete this->vertDescr;
      this->vertDescr = (StructureDescriptor*)(vertDescr->copy());
   }
   this->elementSize = vertDescr->getSize();
   free(data);
   data = (char*)malloc(numEntries * elementSize);
   if (data == NULL && numEntries * elementSize > 0) throw PException("PointSet::clearAndSetup() - cannot allocate memory.");
   memset(data, 0, numEntries * elementSize);

   this->allocatedHeight = height;
   boundingBoxValid = false;
}


const StructureDescriptor *DynamicArrayOfStructures::getDescr() const {
   return vertDescr;
}

DynamicArrayOfStructures *DynamicArrayOfStructures::appendFields(const StructureDescriptor *newAttribs) const {
   DynamicArrayOfStructures *result = new DynamicArrayOfStructures();
   StructureDescriptor *newDescr = (StructureDescriptor*)(vertDescr->copy());
   newDescr->mergeAttributes(newAttribs);
   result->clearAndSetup(width, height, newDescr);

   const mpint sourceSize = vertDescr->getSize();
   const mpint destSize = newDescr->getSize();
   const mpint addDestOffset = destSize - sourceSize;
   pAssert(addDestOffset >= 0);
   char *dataDest = result->data;
   char *dataSource = data;
   for (mpcard vPos=0; vPos<numEntries; vPos++){
      for (mpint b=0; b<sourceSize; b++) {
         *dataDest = *dataSource;
         dataDest++;
         dataSource++;
      }
      for (mpint b=sourceSize; b<destSize; b++) {
         *dataDest = 0;
         dataDest++;
      }
      // dataDest += addDestOffset;
   }

   delete newDescr;
   return result;
}


void DynamicArrayOfStructures::changeHeight(const mpcard newheight) {

   // change ad lib
   mpcard factor = 2;

   if (newheight==0)
   {
      clear();
      return;
   }

   if (this->width==0) this->width=1;

   this->elementSize = vertDescr->getSize();
   mpcard allocatedMem = width*allocatedHeight*elementSize;
   mpcard actualMem = width*height*elementSize;
   mpcard neededMem = width*newheight*elementSize;
   
   // got larger && large enough --> exact reallocation (exactly the number of bytes demanded)
   if ((neededMem > allocatedMem*factor) || (this->allocatedHeight==0))
   {
      data = (char*)realloc(data, neededMem);
      if (data == NULL && neededMem > 0) throw PException("PointSet::changeHeight() - cannot reallocate memory.");
      pAssert(data);
      allocatedHeight = newheight;
   }
   // got only a bit larger --> double memory (but no more than 50MB more than needed)
   else if (neededMem > allocatedMem) {
      mpcard incByFactorMem = allocatedMem*factor;
      pAssert(incByFactorMem >= neededMem);

      if (incByFactorMem - neededMem > 50*1024*1024) {
         incByFactorMem = ((neededMem + 50*1024*1024) / elementSize) * elementSize; // round to proper height (integer div!)
      }
      data = (char*)realloc(data, incByFactorMem);
      if (data == NULL && incByFactorMem > 0) throw PException("PointSet::changeHeight() - cannot allocate memory.");

      pAssert(data);

      allocatedHeight = incByFactorMem / elementSize;
   }
   // got smaller && small enough --> exact reallocation (exactly the number of bytes demanded)
   else if (neededMem <= allocatedMem/factor)
   {
      data = (char*)realloc(data, neededMem);
      if (data == NULL && neededMem > 0) throw PException("PointSet::changeHeight() - cannot reallocate memory.");
      pAssert(data);
      allocatedHeight = newheight;                                                                    
   // got only a bit smaller --> zero remaining area
   }

   // zero new mem
   if (neededMem > actualMem) {
      memset(((char*)data)+actualMem, 0, neededMem - actualMem);
   }

   this->height = newheight;
   this->numEntries = width * height;
   boundingBoxValid = false;
}


void DynamicArrayOfStructures::addPoint(const char *data) {

   pAssert(width == 1 || (width == 0 && height == 0));

   changeHeight(height+1);
   if( data)
	setVertex(height-1, data);
   boundingBoxValid = false;
}

void DynamicArrayOfStructures::addPointFront(const char *buffer)
{
    pAssert(width == 1 || (width == 0 && height == 0));

    // create and copy array into something temporary
    mpcard allocatedMem = width * allocatedHeight * vertDescr->getSize();

    char* tmp = new char[allocatedMem];
    memcpy(tmp, getDataPointer(), allocatedMem);

    // change the whole size of the array
    changeHeight(height+1);

    // copy the content of previous memory to the place just right after the first element
    memcpy(getDataPointer() + vertDescr->getSize(), tmp, allocatedMem);
    delete [] tmp;

    // set point data
    setVertex(0, data);
    boundingBoxValid = false;
}


void DynamicArrayOfStructures::deleteRow(const mpcard index)
{
	pAssert(width >= 1);
	pAssert(index < height);
	if (index < height-1)
	{
		if (width == 0) width = 1;

		for (mpcard i=0; i<width; i++)
		{
			setVertex(i, index, getDataPointer(i, height-1));
		}
	}
	changeHeight(height-1);
	boundingBoxValid = false;
}

void DynamicArrayOfStructures::deletePoint(const mpcard index) {
   pAssert(width == 1);
   pAssert(numEntries > 0);
   pAssert(index < numEntries);

   if (index < height-1) {
      setVertex(index, getDataPointer(height-1));
   }
   changeHeight(height-1);
   boundingBoxValid = false;
}

DynamicArrayOfStructures *DynamicArrayOfStructures::subset(const vector<mpcard>& indices)
{
   DynamicArrayOfStructures *result = new DynamicArrayOfStructures();
   const size_t height = indices.size();
   result->clearAndSetup(1, height, getDescr());
   for (size_t i=0; i<height; i++)
      result->setVertex(i, getDataPointer(indices[i]));
   return result;
}

DynamicArrayOfStructures *DynamicArrayOfStructures::copyHorizontalSlice(mpcard y) {
   DynamicArrayOfStructures *result = new DynamicArrayOfStructures();
   const mpcard width = getWidth();
   result->clearAndSetup(width, 1, getDescr());
   for (mpcard i=0; i<width; i++) {
      result->setVertex(i, 0, getDataPointer(i, y));
   }
   return result;
}

DynamicArrayOfStructures *DynamicArrayOfStructures::copyVerticalSlice(mpcard x) {
   DynamicArrayOfStructures *result = new DynamicArrayOfStructures();
   const mpcard height = getHeight();
   result->clearAndSetup(1, height, getDescr());
   for (mpcard i=0; i<height; i++) {
      result->setVertex(0, i, getDataPointer(x, i));
   }
   return result;
}


BoundingBox3f DynamicArrayOfStructures::calcBoundingBox() const {
   pAssert(providesAttribute("position"));
   const AAT POSITION = getAAT("position");
   const mpcard n = getNumEntries();

   BoundingBox3f result;

   if (n<1) {
      result = BoundingBox3f(makeVector3f(0,0,0));
   } else {
      bool resultSet = false;
      for (mpcard i=0; i<n; i++) {
         Vector3f pos = get<float32, 3>(i, POSITION);
         if (!resultSet) {
            result = BoundingBox3f(pos);
            resultSet = true;
         } else {
            result.addPoint(pos);
         }
      }
   };
   return result;
}

BoundingBox3f DynamicArrayOfStructures::getBoundingBox() const {
   if (!boundingBoxValid) {
      lastBoundingBox = calcBoundingBox();
      boundingBoxValid = true;
   }
   return lastBoundingBox;
}

void DynamicArrayOfStructures::invalidateBoundingBox() const
{
   boundingBoxValid = false;
}


DynamicArrayOfStructures::~DynamicArrayOfStructures() {
   free(data);
   delete vertDescr;
}
