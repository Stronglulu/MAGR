//---------------------------------------------------------------------------
#ifndef DynamicArrayOfStructuresH
#define DynamicArrayOfStructuresH
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "PAssert.h"
#include "LinearAlgebra.h"
#include "StructureDescriptor.h"
#include "BoundingBox.h"
//---------------------------------------------------------------------------
#include <string>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------


class DynamicArrayOfStructures : public Persistent {
   GEOX_CLASS(DynamicArrayOfStructures)
 private:
   StructureDescriptor *vertDescr;
   char *data;
   mpcard width;
   mpcard height;
   mpcard numEntries;
   mpcard elementSize;
   mpcard allocatedHeight;

   string DynamicArrayOfStructures::getDescrString() const;

   char * getDataPtr() const;
   friend StructureDescriptor;

   mutable bool boundingBoxValid;
   mutable BoundingBox3f lastBoundingBox;
   BoundingBox3f calcBoundingBox() const;
 
 public:
   DynamicArrayOfStructures();

   virtual void assign(const Object* obj, CopyContextt *context = NULL);
   virtual void write(OutputObjectStream *s) const;
   virtual void read(InputObjectStream *s);

   // for non-parameterized point sets use width == 1; vertDescr will be owned by new Point Set afterwards
   void clearAndSetup(mpcard width, mpcard height, const StructureDescriptor *vertDescr);
   // just remove all points
   void clear();
   const StructureDescriptor *getDescr() const;

   // macro
   inline AttributeAccessToken getAAT(string name) const;
   inline AttributeAccessToken getAAT(string name, card32 numComponents, card32 dataFormat) const;
   inline bool providesAttribute(string name) const;
   inline bool providesAttribute(string name, card32 numComponents, card32 dataFormat) const;


   inline mpcard getWidth() const;
   inline mpcard getHeight() const;
   inline mpcard getNumEntries() const;
   inline mpcard getElementSize() const;

   /// smart resize (double/half allocated size if necessary)
   void changeHeight(const mpcard newheight);
   /// remove single point at given index; assumes width == 1
   void deletePoint(const mpcard index);

   //! remove single row by replacing the row index with the last row and shrinking the height
   void deleteRow(const mpcard index);

   /// add a single point (buffer points to data in format according to current StructureDescriptor); assumes width == 1
   /// new point will become the last element of the array (getNumEntries() - 1)
   void addPoint(const char *buffer);
   /// same as addPoint hoever adds a point as a first element in the array
   void addPointFront(const char *buffer);

   /// copies a row (0..width-1) at line y
   DynamicArrayOfStructures *copyHorizontalSlice(mpcard y);
   /// copies a column (0..height-1) at index x
   DynamicArrayOfStructures *copyVerticalSlice(mpcard x);

   DynamicArrayOfStructures* subset(const vector<mpcard>& indices);

   /// creates copy with new fields. duplicates are avoided: if an attribute name already exists, no new data field is created.
   DynamicArrayOfStructures *appendFields(const StructureDescriptor *newAttribs) const;

   /// invalidate last bounding box, forces bb rebuild on next getBoundingBox()
   void invalidateBoundingBox() const;
	

   BoundingBox3f getBoundingBox() const;

   
   template<class Format, int dim>
   inline StaticVector<Format, dim> get(mpcard x, mpcard y, AAT token) const;
   template<class Format, int dim>
   inline void set(mpcard x, mpcard y, AAT token, const StaticVector<Format, dim> &v);

   template<typename ScalarType>
   inline ScalarType get(mpcard x, mpcard y, AAT token) const;
   template<typename ScalarType>
   inline void set(mpcard x, mpcard y, AAT token, ScalarType v);

   template<class Format, int dim>
   inline StaticVector<Format, dim> get(mpcard i,AAT token) const;
   template<class Format, int dim>
   inline void set(mpcard i, AAT token, const StaticVector<Format, dim> &v);

   template<typename ScalarType>
   inline ScalarType get(mpcard i,AAT token) const;
   template<typename ScalarType>
   inline void set(mpcard i, AAT token, ScalarType v);


	template<class Format, int dim1, int dim2>
	inline StaticMatrix<Format, dim1, dim2> get(mpcard i,AAT token) const;
	template<class Format, int dim1, int dim2>
	inline StaticMatrix<Format, dim1, dim2> get(mpcard x, mpcard y, AAT token) const;
	template<class Format, int dim1, int dim2>
	inline void set(mpcard i, AAT token, const StaticMatrix<Format, dim1, dim2> &m);
	template<class Format, int dim1, int dim2>
	inline void set(mpcard x, mpcard y, AAT token, const StaticMatrix<Format, dim1, dim2> &m);

	inline card8 get1ub(mpcard i,AAT token) const{ return (get<card8,1>(i, token))[0]; }
	inline card8 get1ub(mpcard x, mpcard y, AAT token) const{ return (get<card8,1>(x,y,token))[0]; }
	inline void set1ub(mpcard i, AAT token, const StaticVector<card8,1> &v){ set<card8,1>(i,token, v); }
	inline void set1ub(mpcard i, AAT token, const card8 value ){
		StaticVector<card8,1> v; v[0] = value;
		set<card8,1>(i,token, v);
	}
	inline void set1ub(mpcard x, mpcard y, AAT token, const card8 value ){
		StaticVector<card8,1> v; v[0] = value;
		set<card8,1>(x,y,token, v);
	}

	inline int32 get1i(mpcard i,AAT token) const{ return (get<int32,1>(i, token))[0]; }
	inline int32 get1i(mpcard x, mpcard y, AAT token) const{ return (get<int32,1>(x,y,token))[0]; }
	inline void set1i(mpcard i, AAT token, const StaticVector<int32,1> &v){ set<int32,1>(i,token, v); }
	inline void set1i(mpcard i, AAT token, const int32 value ){
		StaticVector<int32,1> v; v[0] = value;
		set<int32,1>(i,token, v);
	}
	inline void set1i(mpcard x, mpcard y, AAT token, const int32 value ){
		StaticVector<int32,1> v; v[0] = value;
		set<int32,1>(x,y,token, v);
	}

	inline card32 get1u(mpcard i,AAT token) const{ return (get<card32,1>(i, token))[0]; }
	inline card32 get1u(mpcard x, mpcard y, AAT token) const{ return (get<card32,1>(x,y,token))[0]; }
	inline void set1u(mpcard i, AAT token, const StaticVector<card32,1> &v){ set<card32,1>(i,token, v); }
	inline void set1u(mpcard i, AAT token, const card32 value ){
		StaticVector<card32,1> v; v[0] = value;
		set<card32,1>(i,token, v);
	}
	inline void set1u(mpcard x, mpcard y, AAT token, const card32 value ){
		StaticVector<card32,1> v; v[0] = value;
		set<card32,1>(x,y,token, v);
	}

	inline float32 get1f(mpcard i,AAT token) const{ return (get<float32,1>(i, token))[0]; }
	inline float32 get1f(mpcard x, mpcard y, AAT token) const{ return (get<float32,1>(x,y,token))[0]; }
	inline void set1f(mpcard i, AAT token, const StaticVector<float32,1> &v){ set<float32,1>(i,token, v); }
	inline void set1f(mpcard i, AAT token, const float32 value ){
		StaticVector<float32,1> v; v[0] = value;
		set<float32,1>(i,token, v);
	}
	inline void set1f(mpcard x, mpcard y, AAT token, const float32 value ){
		StaticVector<float32,1> v; v[0] = value;
		set<float32,1>(x,y,token, v);
	}

	inline Vector2ub get2ub(mpcard x, mpcard y, AAT token){
		return get<card8,2>(x,y,token);
	}
	inline Vector2ub get2ub(mpcard i, AAT token){
		return get<card8,2>(i,token);
	}
	inline void set2ub(mpcard x, mpcard y, AAT token, const Vector2ub& v ){
		return set<card8,2>(x,y,token,v);
	}
	inline void set2ub(mpcard i, AAT token, const Vector2ub& v){
		return set<card8,2>(i,token,v);
	}

	inline Vector2i get2i(mpcard x, mpcard y, AAT token)const{
		return get<int32,2>(x,y,token);
	}
	inline Vector2i get2i(mpcard i, AAT token) const{
		return get<int32,2>(i,token);
	}
	inline void set2i(mpcard x, mpcard y, AAT token, const Vector2i& v ){
		return set<int32,2>(x,y,token,v);
	}
	inline void set2i(mpcard i, AAT token, const Vector2i& v){
		return set<int32,2>(i,token,v);
	}

	inline Vector2f get2f(mpcard x, mpcard y, AAT token) const{
		return get<float32,2>(x,y,token);
	}
	inline Vector2f get2f(mpcard i, AAT token) const{
		return get<float32,2>(i,token);
	}
	inline void set2f(mpcard x, mpcard y, AAT token, const Vector2f& v ){
		return set<float32,2>(x,y,token,v);
	}
	inline void set2f(mpcard i, AAT token, const Vector2f& v){
		return set<float32,2>(i,token,v);
	}

	inline Vector3ub get3ub(mpcard x, mpcard y, AAT token) const {
		return get<card8,3>(x,y,token);
	}
	inline Vector3ub get3ub(mpcard i,AAT token) const{
		 return get<card8,3>(i, token);
   }
	inline void set3ub(mpcard x, mpcard y, AAT token, const Vector3ub &v){
		 set<card8,3>(x,y,token, v);
	}
	inline void set3ub(mpcard i, AAT token, const Vector3ub &v){
		 set<card8,3>(i,token, v);
	}

	inline Vector3i get3i(mpcard x, mpcard y, AAT token) const {
		return get<int32,3>(x,y,token);
	}
	inline Vector3i get3i(mpcard i,AAT token) const{
		 return get<int32,3>(i, token);
   }
	inline void set3i(mpcard x, mpcard y, AAT token, const Vector3i &v){
		 set<int32,3>(x,y,token, v);
	}
	inline void set3i(mpcard i, AAT token, const Vector3i &v){
		 set<int32,3>(i,token, v);
	}

	inline Vector3f get3f(mpcard x, mpcard y, AAT token) const {
		return get<float32,3>(x,y,token);
	}
	inline Vector3f get3f(mpcard i,AAT token) const{
		 return get<float32,3>(i, token);
   }
	inline void set3f(mpcard x, mpcard y, AAT token, const Vector3f &v){
		 set<float32,3>(x,y,token, v);
	}
	inline void set3f(mpcard i, AAT token, const Vector3f &v){
		 set<float32,3>(i,token, v);
	}

	inline Vector4ub get4ub(mpcard x, mpcard y, AAT token) const {
		return get<card8,4>(x,y,token);
	}
	inline Vector4ub get4ub(mpcard i,AAT token) const{
		 return get<card8,4>(i, token);
   }
	inline void set4ub(mpcard x, mpcard y, AAT token, const Vector4ub &v){
		 set<card8,4>(x,y,token, v);
	}
	inline void set4ub(mpcard i, AAT token, const Vector4ub &v){
		 set<card8,4>(i,token, v);
	}

	inline Vector4i get4i(mpcard x, mpcard y, AAT token) const {
		return get<int32,4>(x,y,token);
	}
	inline Vector4i get4i(mpcard i,AAT token) const{
		 return get<int32,4>(i, token);
   }
	inline void set4i(mpcard x, mpcard y, AAT token, const Vector4i &v){
		 set<int32,4>(x,y,token, v);
	}
	inline void set4i(mpcard i, AAT token, const Vector4i &v){
		 set<int32,4>(i,token, v);
	}

	inline Vector4f get4f(mpcard x, mpcard y, AAT token) const {
		return get<float32,4>(x,y,token);
	}
	inline Vector4f get4f(mpcard i,AAT token) const{
		 return get<float32,4>(i, token);
   }
	inline void set4f(mpcard x, mpcard y, AAT token, const Vector4f &v){
		 set<float32,4>(x,y,token, v);
	}
	inline void set4f(mpcard i, AAT token, const Vector4f &v){
		 set<float32,4>(i,token, v);
	}

	inline Matrix3f get9f(mpcard x, mpcard y, AAT token) const {
		return get<float32,3,3>(x,y,token);
	}
	inline Matrix3f get9f(mpcard i,AAT token) const{
		return get<float32,3,3>(i, token);
	}
	inline void set9f(mpcard x, mpcard y, AAT token, const Matrix3f &m){
		set<float32,3,3>(x,y,token, m);
	}
	inline void set9f(mpcard i, AAT token, const Matrix3f &m){
		set<float32,3,3>(i,token, m);
	}

   inline void setVertex(mpcard x, mpcard y, const char *buffer);

   inline void setVertex(mpcard index, const char *buffer);

   inline void getVertex(mpcard x, mpcard y, char *buffer) const;

   inline void getVertex(mpcard index, char *buffer) const;

   inline char *getDataPointer(mpcard index = 0);

   inline const char *getDataPointer(mpcard index = 0) const;

   inline char *getDataPointer(mpcard x, mpcard y);

   inline const char *getDataPointer(mpcard x, mpcard y) const;

   ~DynamicArrayOfStructures();
};

typedef DynamicArrayOfStructures DAOS;



#endif
