//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "GLGeometryViewer.h"
//---------------------------------------------------------------------------
#include "ExceptionHandling.h"
//---------------------------------------------------------------------------

GLGeometryViewer::GLGeometryViewer(QWidget * parent, Qt::WindowFlags f)
: QWidget(parent, f)
{
   ui.setupUi(this);
	connect(ui.glFrame,SIGNAL(widgetContentChanged()),this,SLOT(sltWidgetContentChanged()));
	connect(ui.glFrame,SIGNAL(pointSelected(int)),this,SLOT(sltPointSelected(int)));

	cbPointSelected = NULL;
}

GLGeometryViewer::~GLGeometryViewer()
{
}

int GLGeometryViewer::addPoint(Point2D p)
{
	return ui.glFrame->addPoint(p);
}

Point2D GLGeometryViewer::getPoint(int index)
{
	return ui.glFrame->getPoint(index);
}

void GLGeometryViewer::setPoint(int index, Point2D p)
{
	ui.glFrame->setPoint(index,p);
}


int GLGeometryViewer::addLine(Line l)
{
	return ui.glFrame->addLine(l);
}

int GLGeometryViewer::addLine(Vector2f pos1, Vector2f pos2)
{
	Line l;
	l.vertices[0] = ui.glFrame->addPoint(Point2D(pos1));
	l.vertices[1] = ui.glFrame->addPoint(Point2D(pos2));
	return ui.glFrame->addLine(l);
}

int GLGeometryViewer::addLine(float pos1X, float pos1Y, float pos2X, float pos2Y)
{
	Line l;
	l.vertices[0] = ui.glFrame->addPoint(Point2D(pos1X,pos1Y));
	l.vertices[1] = ui.glFrame->addPoint(Point2D(pos2X,pos2Y));
	return ui.glFrame->addLine(l);
}

Line GLGeometryViewer::getLine(int index)
{
	return ui.glFrame->getLine(index);
}

void GLGeometryViewer::setLine(int index, Line l)
{
	ui.glFrame->setLine(index,l);
}

int GLGeometryViewer::addTriangle(Triangle t)
{
	return ui.glFrame->addTriangle(t);
}

int GLGeometryViewer::addTriangle(Vector2f pos1, Vector2f pos2, Vector2f pos3)
{
	Triangle t;
	t.vertices[0] = ui.glFrame->addPoint(Point2D(pos1));
	t.vertices[1] = ui.glFrame->addPoint(Point2D(pos2));
	t.vertices[2] = ui.glFrame->addPoint(Point2D(pos3));
	return ui.glFrame->addTriangle(t);
}

int GLGeometryViewer::addTriangle(float pos1X, float pos1Y, float pos2X, float pos2Y, float pos3X, float pos3Y)
{
	Triangle t;
	t.vertices[0] = ui.glFrame->addPoint(Point2D(pos1X,pos1Y));
	t.vertices[1] = ui.glFrame->addPoint(Point2D(pos2X,pos2Y));
	t.vertices[2] = ui.glFrame->addPoint(Point2D(pos3X,pos3Y));
	return ui.glFrame->addTriangle(t);
}

Triangle GLGeometryViewer::getTriangle(int index)
{
	return ui.glFrame->getTriangle(index);
}

void GLGeometryViewer::setTriangle(int index, Triangle t)
{
	ui.glFrame->setTriangle(index,t);
}

void GLGeometryViewer::clear()
{
	ui.glFrame->clear();
}

void GLGeometryViewer::clearLines()
{
  ui.glFrame->clearLines();
}

void GLGeometryViewer::clearTriangles()
{
  ui.glFrame->clearTriangles();
}

void GLGeometryViewer::on_btnCamera_pressed()
{
	ui.glFrame->setCameraMode();
}
void GLGeometryViewer::on_btnPick_pressed()
{
	ui.glFrame->setPickMode();
}
void GLGeometryViewer::on_btnClear_pressed()
{
	ui.glFrame->clear();
}
void GLGeometryViewer::on_btnCreate_pressed()
{
	ui.glFrame->setCreateMode();
}

void GLGeometryViewer::sltWidgetContentChanged()
{
	emit viewerContentChanged();
}

void GLGeometryViewer::sltPointSelected(int index)
{
	emit pointSelected(index);
	if (cbPointSelected) cbPointSelected(index);
}

#ifdef _WIN32
#include "moc/GLGeometryViewer_moc.cpp"
#else
#include "moc_GLGeometryViewer.cpp"
#endif

