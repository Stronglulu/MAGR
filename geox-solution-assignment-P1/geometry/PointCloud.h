//---------------------------------------------------------------------------
#ifndef PointCloudH
#define PointCloudH
//---------------------------------------------------------------------------
#include "DynamicArrayOfStructures.h"
//---------------------------------------------------------------------------


class BasicUnstructeredInCorePointCloudIterator;

/// a simple point cloud (list of points)
class  PointCloud : public Persistent {
   GEOX_CLASS(PointCloud)
 protected:
   DynamicArrayOfStructures *ps;

 public:
   PointCloud();

   virtual void assign(const Object* obj, CopyContextt *context = NULL);

   AAT getAAT(string name) const {return ps->getAAT(name);}
   virtual const StructureDescriptor *getDescr() const;
   virtual card64 getNumPoints() const;
   virtual void  clear();

   // --- new methods
   void clearAndSetup(const StructureDescriptor *vertDescr, mpcard numPts);

   /// shortcut for one of the most commonly used setup
   void clearAndSetup(mpcard numPoints, bool hasPosition=true, bool hasNormal=true, bool hasColor=true, bool hasFlags=true);

   virtual const DynamicArrayOfStructures *getVertices() const {return ps;}
   virtual void setVertices(DynamicArrayOfStructures *ps) {delete this->ps; this->ps = ps;}
   virtual DynamicArrayOfStructures *getVertices() {return ps;}
   virtual PointCloud *createCopyWithNewPointSet(DynamicArrayOfStructures *ps) const; // ownership of ps will be transfered to this

   // ---

   Vector3f computeCenterOfMass() const;

   ~PointCloud();
};





#endif
