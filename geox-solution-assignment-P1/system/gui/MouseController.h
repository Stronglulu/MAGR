//---------------------------------------------------------------------------
#ifndef MouseControllerH
#define MouseControllerH
//---------------------------------------------------------------------------
#include "MouseButtons.h"
#include "Modifiers.h"
#include "PTypes.h"
//---------------------------------------------------------------------------



class  MouseController { /* INTERFACE */
 public:
   virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
	virtual void mouseMoved(int32 x, int32 y) {}
	virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
	virtual void mouseWheelRotated(int32 rotatedDelta,  Modifiers modifiersState) {}
	virtual void areaResize(card32 width, card32 height) {}
   virtual ~MouseController() {}
};



#endif
