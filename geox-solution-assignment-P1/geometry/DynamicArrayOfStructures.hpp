//---------------------------------------------------------------------------
#ifndef PointSet_templCodeH
#define PointSet_templCodeH
//---------------------------------------------------------------------------
#include "DynamicArrayOfStructures.h"
#include "StructureDescriptor.hpp"
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------


inline char *DynamicArrayOfStructures::getDataPtr() const {
   return data;
}

inline AttributeAccessToken DynamicArrayOfStructures::getAAT(string name) const {
   return getDescr()->getToken(name);
}

inline AttributeAccessToken DynamicArrayOfStructures::getAAT(string name, card32 numComponents, card32 dataFormat) const {
   return getDescr()->getToken(name, numComponents, dataFormat);
}

inline bool DynamicArrayOfStructures::providesAttribute(string name) const {
   return getDescr()->providesAttribute(name);
}

inline bool DynamicArrayOfStructures::providesAttribute(string name, card32 numComponents, card32 dataFormat) const {
   return getDescr()->providesAttribute(name, numComponents, dataFormat);
}

inline mpcard DynamicArrayOfStructures::getWidth() const {
   return width;
}

inline mpcard DynamicArrayOfStructures::getHeight() const {
   return height;
}

inline mpcard DynamicArrayOfStructures::getNumEntries() const {
   return numEntries;
}

inline mpcard DynamicArrayOfStructures::getElementSize() const {
   return elementSize;
}

template<class Format, int dim>
inline StaticVector<Format, dim> DynamicArrayOfStructures::get(mpcard x, mpcard y, AAT token) const {
   #ifdef _DEBUG
   pAssert(sizeof(StaticVector<Format, dim>) == token.size);
   #endif
   pAssert(x<width && y < height);
   char *pos = data + (x+width*y)*elementSize  + token.offset;
   return *((StaticVector<Format, dim>*)(pos));
}

template<class Format, int dim>
inline void DynamicArrayOfStructures::set(mpcard x, mpcard y, AAT token, const StaticVector<Format, dim> &v) {
   #ifdef _DEBUG
   pAssert(sizeof(StaticVector<Format, dim>) == token.size);
   #endif
   pAssert(x<width && y < height);
   char *pos = data + (x+width*y)*elementSize  + token.offset;
   *((StaticVector<Format, dim>*)(pos)) = v;
   boundingBoxValid = false;
}

template<class Format, int dim>
inline StaticVector<Format, dim> DynamicArrayOfStructures::get(mpcard i,AAT token) const {
   #ifdef _DEBUG
   pAssert(sizeof(StaticVector<Format, dim>) == token.size);
   #endif
   pAssert(i<numEntries);
   char *pos = data + i*elementSize  + token.offset;
   return *((StaticVector<Format, dim>*)(pos));
}

template<class Format, int dim>
inline void DynamicArrayOfStructures::set(mpcard i, AAT token, const StaticVector<Format, dim> &v) {
   #ifdef _DEBUG
   pAssert(sizeof(StaticVector<Format, dim>) == token.size);
   #endif
   pAssert(i<numEntries);
   char *pos = data + i*elementSize  + token.offset;
   *((StaticVector<Format, dim>*)(pos)) = v;
   boundingBoxValid = false;
}




template<typename ScalarType>
inline ScalarType DynamicArrayOfStructures::get(mpcard x, mpcard y, AAT token) const {
   #ifdef _DEBUG
   pAssert(sizeof(ScalarType) == token.size);
   #endif
   pAssert(x<width && y < height);
   char *pos = data + (x+width*y)*elementSize  + token.offset;
   return *((ScalarType*)(pos));
}

template<typename ScalarType>
inline void DynamicArrayOfStructures::set(mpcard x, mpcard y, AAT token, ScalarType v) {
   #ifdef _DEBUG
   pAssert(sizeof(ScalarType) == token.size);
   #endif
   pAssert(x<width && y < height);
   char *pos = data + (x+width*y)*elementSize  + token.offset;
   *((ScalarType*)(pos)) = v;
   boundingBoxValid = false;
}

template<typename ScalarType>
inline ScalarType DynamicArrayOfStructures::get(mpcard i,AAT token) const {
   #ifdef _DEBUG
   pAssert(sizeof(ScalarType) == token.size);
   #endif
   pAssert(i<numEntries);
   char *pos = data + i*elementSize  + token.offset;
   return *((ScalarType*)(pos));
}

template<typename ScalarType>
inline void DynamicArrayOfStructures::set(mpcard i, AAT token, ScalarType v) {
   #ifdef _DEBUG
   pAssert(sizeof(ScalarType) == token.size);
   #endif
   pAssert(i<numEntries);
   char *pos = data + i*elementSize  + token.offset;
   *((ScalarType*)(pos)) = v;
   boundingBoxValid = false;
}


template<class Format, int dim1, int dim2>
inline StaticMatrix<Format, dim1, dim2> DynamicArrayOfStructures::get(mpcard i,AAT token) const {
	#ifdef _DEBUG
	pAssert(sizeof(StaticMatrix<Format, dim1, dim2>) == token.size);
	#endif
	char *pos = data + i*elementSize  + token.offset;
	return *((StaticMatrix<Format, dim1, dim2>*)pos);
}

template<class Format, int dim1, int dim2>
inline StaticMatrix<Format, dim1, dim2> DynamicArrayOfStructures::get(mpcard x, mpcard y, AAT token) const {
	#ifdef _DEBUG
	pAssert(sizeof(StaticMatrix<Format, dim1, dim2>) == token.size);
	#endif
	char *pos = data + (x+width*y)*elementSize  + token.offset;
	return *((StaticMatrix<Format, dim1, dim2>*)pos);
}

template<class Format, int dim1, int dim2>
inline void DynamicArrayOfStructures::set(mpcard i, AAT token, const StaticMatrix<Format, dim1, dim2> &m) {
	#ifdef _DEBUG
	pAssert(sizeof(StaticMatrix<Format, dim1, dim2>) == token.size);
	#endif
	char *pos = data + i*elementSize  + token.offset;
	*((StaticMatrix<Format, dim1, dim2>*)pos) = m;
}

template<class Format, int dim1, int dim2>
inline void DynamicArrayOfStructures::set(mpcard x, mpcard y, AAT token, const StaticMatrix<Format, dim1, dim2> &m) {
	#ifdef _DEBUG
	pAssert(sizeof(StaticMatrix<Format, dim1, dim2>) == token.size);
	#endif
	char *pos = data + (x+width*y)*elementSize  + token.offset;
	*((StaticMatrix<Format, dim1, dim2>*)pos) = m;
}




inline void DynamicArrayOfStructures::setVertex(mpcard x, mpcard y, const char *buffer) {
   char *pos = data + (x+width*y)*elementSize;
   for (mpcard i=0; i<elementSize; i++) {
      pos[i] = buffer[i];
   }
   boundingBoxValid = false;
}

inline void DynamicArrayOfStructures::setVertex(mpcard index, const char *buffer) {
   char *pos = data + index*elementSize;
   for (mpcard i=0; i<elementSize; i++) {
      pos[i] = buffer[i];
   }
   boundingBoxValid = false;
}

inline void DynamicArrayOfStructures::getVertex(mpcard x, mpcard y, char *buffer) const {
   char *pos = data + (x+width*y)*elementSize;
   for (mpcard i=0; i<elementSize; i++) {
      buffer[i] = pos[i];
   }
}

inline void DynamicArrayOfStructures::getVertex(mpcard index, char *buffer) const {
   char *pos = data + index*elementSize;
   for (mpcard i=0; i<elementSize; i++) {
      buffer[i] = pos[i];
   }
}

inline char *DynamicArrayOfStructures::getDataPointer(mpcard index) {
   boundingBoxValid = false;
   return data + index*elementSize;
}

inline const char *DynamicArrayOfStructures::getDataPointer(mpcard index) const {
   return data + index*elementSize;
}

inline char *DynamicArrayOfStructures::getDataPointer(mpcard x, mpcard y) {
   boundingBoxValid = false;
   return data + (x+width*y)*elementSize;
}

inline const char *DynamicArrayOfStructures::getDataPointer(mpcard x, mpcard y) const {
   return data + (x+width*y)*elementSize;
}




#endif
