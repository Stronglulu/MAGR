//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "BasicGLViewer3D.h"
//---------------------------------------------------------------------------
#include "ExceptionHandling.h"
#include "ExaminerCameraController.h"
//---------------------------------------------------------------------------

BasicGLViewer3D::BasicGLViewer3D(QWidget * parent, Qt::WindowFlags f)
: QWidget(parent, f)
{
   ui.setupUi(this);
	connect(ui.glFrame,SIGNAL(widgetContentChanged()),this,SLOT(sltWidgetContentChanged()));
	connect(ui.glFrame,SIGNAL(customPaintGL()),this,SLOT(sltCustomRenderGL()));
}

BasicGLViewer3D::~BasicGLViewer3D()
{
}



void BasicGLViewer3D::on_btnResetCamera_pressed()
{
	ui.glFrame->resetCamera();
	ui.glFrame->updateGL();
}

void BasicGLViewer3D::sltWidgetContentChanged()
{
	emit viewerContentChanged();
}

void BasicGLViewer3D::on_btnDrawAxes_clicked(bool down)
{
	ui.glFrame->repaint();
}

void BasicGLViewer3D::on_btnOrtho_clicked(bool down)
{
	ui.glFrame->setOrthoView(down); 
	ui.glFrame->repaint();
}

void BasicGLViewer3D::sltCustomRenderGL()
{
	if (ui.btnDrawAxes->isChecked()) drawAxes();
	for (size_t i=0; i<renderables.size(); i++) {
		renderables[i]->renderGL();
	}
}

void BasicGLViewer3D::drawAxes()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glDisable(GL_LIGHTING);
	glLineWidth(1.0f);

	glBegin(GL_LINES);

	glColor4f(1.0f,0.0f,0.0f,1.0f);
	glVertex3f(-2000.0f, 0.0f, 0.0f);
	glVertex3f( 2000.0f, 0.0f, 0.0f);

	glColor4f(0.0f,1.0f,0.0f,1.0f);
	glVertex3f(0.0f, -2000.0f, 0.0f);
	glVertex3f(0.0f,  2000.0f, 0.0f);

	glColor4f(0.0f,0.0f,1.0f,1.0f);
	glVertex3f(0.0f, 0.0f, -2000.0f);
	glVertex3f(0.0f, 0.0f,  2000.0f);

	glColor4f(0.8f,0.8f,0.8f,1.0f);
	glVertex3f(-1.0f,-1.0f,-1.0f);
	glVertex3f( 1.0f,-1.0f,-1.0f);

	glVertex3f( 1.0f,-1.0f,-1.0f);
	glVertex3f( 1.0f, 1.0f,-1.0f);

	glVertex3f( 1.0f, 1.0f,-1.0f);
	glVertex3f(-1.0f, 1.0f,-1.0f);

	glVertex3f(-1.0f, 1.0f,-1.0f);
	glVertex3f(-1.0f,-1.0f,-1.0f);

	glVertex3f(-1.0f,-1.0f, 1.0f);
	glVertex3f( 1.0f,-1.0f, 1.0f);

	glVertex3f( 1.0f,-1.0f, 1.0f);
	glVertex3f( 1.0f, 1.0f, 1.0f);

	glVertex3f( 1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);

	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f,-1.0f, 1.0f);

	glVertex3f(-1.0f,-1.0f,-1.0f);
	glVertex3f(-1.0f,-1.0f,-1.0f);

	glVertex3f(-1.0f,-1.0f,-1.0f);
	glVertex3f(-1.0f, 1.0f,-1.0f);

	glVertex3f(-1.0f, 1.0f,-1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);

	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f,-1.0f, 1.0f);

	glVertex3f(-1.0f,-1.0f, 1.0f);
	glVertex3f(-1.0f,-1.0f,-1.0f);

	glVertex3f( 1.0f,-1.0f,-1.0f);
	glVertex3f( 1.0f, 1.0f,-1.0f);

	glVertex3f( 1.0f, 1.0f,-1.0f);
	glVertex3f( 1.0f, 1.0f, 1.0f);

	glVertex3f( 1.0f, 1.0f, 1.0f);
	glVertex3f( 1.0f,-1.0f, 1.0f);

	glVertex3f( 1.0f,-1.0f, 1.0f);
	glVertex3f( 1.0f,-1.0f,-1.0f);

	glEnd();

	glPopAttrib();

}

void BasicGLViewer3D::on_btnYAxis_clicked(bool down)
{
	ExaminerCameraController *cc = dynamic_cast<ExaminerCameraController*>(ui.glFrame->getCameraController());
	if (cc) cc->setRotAroundZAxis(!down);
	ui.glFrame->repaint();
}

#ifdef _WIN32
#include "moc/BasicGLViewer3D_moc.cpp"
#else
#include "moc_GLGeometryViewer3DImproved.cpp"
#endif

