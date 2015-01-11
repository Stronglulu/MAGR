	//---------------------------------------------------------------------------
#ifndef StructureDescriptorH
#define StructureDescriptorH
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "PAssert.h"
#include "LinearAlgebra.h"
#include "DynamicLinearAlgebra.h"
#include "DynamicLinearAlgebra.hpp"
//---------------------------------------------------------------------------
#include <string>
#include "AttributeAccessToken.h"
#include "StructureAttribDescriptor.h"
//---------------------------------------------------------------------------

class DynamicArrayOfStructures;


/// structure of an array of structs
class  StructureDescriptor : public Persistent {
 private:
   GEOX_CLASS(StructureDescriptor)

   /// list of attributes
   vector<StructureAttribDescriptor> attributes;
   /// offset in memory to element i (contains attributes.size() + 1 offsets, first element always == 0)
   vector<card32> offsets;

 public:
   StructureDescriptor();

   virtual void assign(const Object* obj, CopyContextt *context = NULL);
   virtual bool equal(const Object* obj) const;
   bool operator==(const StructureDescriptor *vd) const;
   bool operator!=(const StructureDescriptor *vd) const;

   virtual void write(OutputObjectStream *s) const;
   virtual void read(InputObjectStream *s);

   void   pushAttrib(StructureAttribDescriptor a);
   void   clear();
   mpcard getNumAttribs() const;
   mpcard getNumAttributes() const;
   inline StructureAttribDescriptor getAttribute(card32 i) const;
   inline StructureAttribDescriptor getAttribute(string name) const;
   card32 getIndex(string name) const;
   bool   providesAttribute(string name) const;
   bool   providesAttribute(string name, card32 numComponents) const;
   bool   providesAttribute(string name, card32 numComponents, card32 dataFormat) const;
   bool   providesAttribute(SAD att) const;
   /// returns size of a single vertex in memory (in bytes)
   card32 getSize() const;
   string getDescription() const;

   /// incorporate new attributes into current descriptor,
   /// duplicate fields (field with the same name) are avoided,
   /// an exception is thrown if type of duplicate fields does not match
   void mergeAttributes(const StructureDescriptor *newAttribs);

   AttributeAccessToken getToken(string name) const;
	/// gets aat with given name; also checks for other attributes and throws exception if they do not match
   AttributeAccessToken getToken(string name, card32 numComponents, card32 dataFormat) const;
   AttributeAccessToken getToken(card32 i) const;
   inline card32 getAttributeOffset(card32 i) const;

   /// returns the number of all scalar channels (sum of dimensions of channels)
   inline mpcard computeOverallDimension() const;
   
};





#endif
