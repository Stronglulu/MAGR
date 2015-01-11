//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "InitGeoX.h"
//---------------------------------------------------------------------------
#include "GeoXMainWindow.h"
#include "ExceptionHandling.h"
#include "LogoViewer.h"
#include "GeoXOutput.h"
//---------------------------------------------------------------------------
// include icons & other resources
#include "../images/moc/geoX_rcc.h"
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------



int APIENTRY _tWinMain(HINSTANCE hInstance,
                       HINSTANCE hPrevInstance,
                       LPTSTR    lpCmdLine,
                       int       nCmdShow)
{
   UNREFERENCED_PARAMETER(hInstance);
   UNREFERENCED_PARAMETER(hPrevInstance);
   UNREFERENCED_PARAMETER(lpCmdLine);
   UNREFERENCED_PARAMETER(nCmdShow);

   setStructureExceptionHandler();

   int n = 0;
   QApplication qapp(n, 0);

   init();
   qInitResources();

   //QApplication::setStyle(new QCleanlooksStyle); 
   //QApplication::setStyle(new QPlastiqueStyle);

   // create main window	
   mainWindow = new GeoXMainWindow;
   mainWindow->show();
   mainWindow->setDisplayWidget(new LogoViewer());
   qapp.setActiveWindow( mainWindow );

   //Close all windows with main window
   QApplication::connect(mainWindow, SIGNAL(destroyed()), &qapp, SLOT(closeAllWindows()));

   output << "GeoX started.\n";

   // run it
   qapp.exec();

   shutdown();
   qCleanupResources();

   return 0;
}

