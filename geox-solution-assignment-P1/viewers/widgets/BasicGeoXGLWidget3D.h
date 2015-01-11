//---------------------------------------------------------------------------
#ifndef BasicGeoXGLWidget3DH
#define BasicGeoXGLWidget3DH
//---------------------------------------------------------------------------
#include <QtOpenGL/QGLWidget>
#include <GL/glu.h>
#include "ExaminerCameraController.h"
#include "Camera.h"
#include "ViewFrustum.h"
#include <QtWidgets/QWidget.h>
#include <QtOpenGL/qgl.h>
//---------------------------------------------------------------------------

class BasicGeoXGLWidget3D : public QGLWidget {
   Q_OBJECT

 private:
	Camera camera;
	ViewFrustum vf;
	CameraController* camControl;

	Vector3f bgColor;
	bool orthoView;

 signals:
	void widgetContentChanged();
	void customPaintGL();

 protected:
	virtual void paintGL();
	virtual void initializeGL();
	virtual void resizeGL(int width, int height);
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent (QMouseEvent * event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void wheelEvent(QWheelEvent * event);

 public:
   BasicGeoXGLWidget3D(QWidget *parent=0, const QGLWidget *shareWidget=0, Qt::WindowFlags f=0);
	
	void resetCamera();       // set camera to initial position
	
	void setBackgroundColor(Vector3f color){bgColor = color;};
	void setBackgroundColor(float red, float green, float blue){bgColor = makeVector3f(red,green,blue);};

	bool getOrthoView() const { return orthoView; }
	void setOrthoView(bool val) { orthoView = val; }

	CameraController *getCameraController() {return camControl;}

	virtual ~BasicGeoXGLWidget3D();
};

#endif
