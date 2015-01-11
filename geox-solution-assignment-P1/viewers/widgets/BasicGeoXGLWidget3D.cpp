//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "BasicGeoXGLWidget3D.h"
#include <QtGui/qevent.h>
#include "GeoXOutput.h"
//---------------------------------------------------------------------------



BasicGeoXGLWidget3D::BasicGeoXGLWidget3D(QWidget *parent, const QGLWidget *shareWidget, Qt::WindowFlags f)
: QGLWidget(parent, shareWidget, f)
{
	initializeGL();
	camControl = new ExaminerCameraController();
	camControl->bindCamera(&camera);
	camControl->setNotifyOnCameraChange(QDelegate(this, &BasicGeoXGLWidget3D::updateGL));
	bgColor = makeVector3f(0.2f,0.2f,0.4f);
	orthoView = false;
}

void BasicGeoXGLWidget3D::initializeGL() 
{
	camera.setPosition(makeVector3f(0.0f,0.0f,10.0f));
	camera.setLookAt(makeVector3f(0.0f,0.0f,0.0f));
	camera.setUp(makeVector3f(0.0f,1.0f,0.0f));
	glViewport(0,0,width(),height());
}


void BasicGeoXGLWidget3D::resetCamera()
{
	camControl->setup(makeVector3f(0.0f,0.0f,10.0f),makeVector3f(0.0f,0.0f,0.0f),makeVector3f(0.0f,1.0f,0.0f));
}




void BasicGeoXGLWidget3D::paintGL()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float32 aspect;
	if (height() > 0) {
		aspect = ((float)width())/((float)height());
	} else {
		aspect = 1;
	}
	vf.setXToYAspectRatio(aspect);
	if (orthoView)
		glOrtho(-10.0, 10.0, -10.0, 10.0, vf.getZNearClip(), vf.getZFarClip());
	else
		gluPerspective(vf.getVerticalFieldOfView(), aspect, vf.getZNearClip(), vf.getZFarClip());
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	Vector3f pos = camera.getPosition();
	Vector3f lookAt = camera.getLookAt();
	Vector3f up = camera.getUp();
	gluLookAt(pos[0],pos[1],pos[2],lookAt[0],lookAt[1],lookAt[2],up[0],up[1],up[2]);
	glClearColor(bgColor[0],bgColor[1],bgColor[2],0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	emit customPaintGL();
}

void BasicGeoXGLWidget3D::mousePressEvent(QMouseEvent *event)
{
	camControl->mouseDown(event->x(),event->y(),MouseButtons(event->button()==Qt::LeftButton,event->button()==Qt::RightButton, event->button()==Qt::MidButton),event->modifiers());

	updateGL();
}

void BasicGeoXGLWidget3D::mouseReleaseEvent (QMouseEvent * event)
{
	camControl->mouseUp(event->x(),event->y(),MouseButtons(event->button()==Qt::LeftButton,event->button()==Qt::RightButton, event->button()==Qt::MidButton),event->modifiers());

	updateGL();
}

void BasicGeoXGLWidget3D::mouseMoveEvent(QMouseEvent *event)
{
	camControl->mouseMoved(event->x(),event->y());

	updateGL();
}

void BasicGeoXGLWidget3D::resizeGL(int width, int height)
{
	this->resize(width,height);
	glViewport(0,0,this->width(),this->height());
}

void BasicGeoXGLWidget3D::wheelEvent(QWheelEvent * event)
{
	camControl->mouseWheelRotated(event->delta(), Modifiers(event->modifiers()));
	updateGL();
}

BasicGeoXGLWidget3D::~BasicGeoXGLWidget3D()
{
}



#ifdef _WIN32
#include "moc/BasicGeoXGLWidget3D_moc.cpp"
#else
#include "moc_BasicGeoXGLWidget3D.cpp"
#endif

