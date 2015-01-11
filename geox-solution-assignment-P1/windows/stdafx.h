// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#ifdef _WIN32

#pragma once

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>
#include <mmsystem.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>


// TODO: reference additional headers your program requires here
#define GEOX_PRECOMPILE_A_LOT
#ifdef GEOX_PRECOMPILE_A_LOT

   #include <gl/gl.h>
   #include <gl/glu.h>

   #include <stdlib.h>
   #include <stdio.h>
   #include <math.h>
   #include <values_vs.h>
   #include <iostream>
   #include <stack>
   #include <string>
   #include <list>
   #include <deque>
   #include <algorithm>
   #include <vector>
   #include <map>

   #include <QtCore/Qt>
   #include <QtCore/qnamespace.h>
   #include <QtCore/QDir>
   #include <QtCore/QString>
   #include <QtCore/QTimer>
   #include <QtWidgets/QWidget.h>
   #include <QtWidgets/QApplication>
   #include <QtGui/QBitmap>
   #include <QtWidgets/QBoxLayout>
   #include <QtWidgets/QComboBox>
   #include <QtWidgets/QDesktopWidget>
   #include <QtWidgets/QDialog>
   #include <QtGui/qevent.h>
   #include <QtWidgets/QFileDialog>
   #include <QtWidgets/QFrame>
   #include <QtWidgets/QHBoxLayout>
   #include <QtGui/QIcon>
   #include <QtGui/QImage>
   #include <QtWidgets/QInputDialog>
   #include <QtGui/QKeyEvent>
   #include <QtWidgets/QLabel>
   #include <QtWidgets/QLineEdit>
   #include <QtWidgets/QMenu>
   #include <QtWidgets/QMessageBox>
   #include <QtGui/QPalette>
   #include <QtGui/QPixmap>
   #include <QtWidgets/QPushButton>
   #include <QtWidgets/QScrollArea.h>
   #include <QtWidgets/QSpacerItem>
   #include <QtWidgets/QSplitter>
   #include <QtWidgets/QTreeWidget>
   #include <QtWidgets/QVBoxLayout>


   #include "HashTables.h"
   #include "Tools.h"
   #include "PTypes.h"
   #include "MetaClass.h"
   #include "Object.h"
   #include "Persistent.h"
   #include "LinearAlgebra.h"
   #include "LinearAlgebra.hpp"
   #include "Exceptions.h"
   #include "pAssert.h"
   #include "ObjectClassProperty.h"
   #include "EmbeddedObjectClassProperty.h"
   #include "StringHelper.h"
   #include "NumericalClassProperty.h"
   #include "BooleanClassProperty.h"
   #include "StringClassProperty.h"
   #include "FixedArrayClassProperty.h"
   #include "VariableArrayClassProperty.h"
   #include "HalfSpace.h"
   #include "SparseLinearAlgebra.h"
   #include "SparseLinearAlgebra.hpp"
   #include "DynamicLinearAlgebra.h"
   #include "DynamicLinearAlgebra.hpp"
   #include "ObjectStreams.h"
   #include "ObjectViewsTable.h"
   #include "GeoXOutput.h"
   #include "BoundingBox.h"
   #include "BoundingBox.hpp"

#endif

#endif
