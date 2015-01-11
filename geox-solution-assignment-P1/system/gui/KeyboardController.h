//---------------------------------------------------------------------------
#ifndef KeyboardControllerH
#define KeyboardControllerH
//---------------------------------------------------------------------------
#include "GeneralKey.h"
//---------------------------------------------------------------------------


class KeyboardController { /* INTERFACE */
 public:
	virtual void keyDown(GeneralKey key) {};
	virtual void keyUp(GeneralKey key) {};
   virtual ~KeyboardController() {}
};


#endif
