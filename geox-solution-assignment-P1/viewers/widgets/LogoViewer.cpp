//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "LogoViewer.h"
//---------------------------------------------------------------------------
#include "ExceptionHandling.h"
//---------------------------------------------------------------------------

LogoViewer::LogoViewer(QWidget * parent, Qt::WindowFlags f)
: QWidget(parent, f)
{
   ui.setupUi(this);
}

LogoViewer::~LogoViewer()
{
}

#ifdef _WIN32
#include "moc/LogoViewer_moc.cpp"
#else
#include "moc_LogoViewer.cpp"
#endif

