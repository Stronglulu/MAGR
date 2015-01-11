//---------------------------------------------------------------------------
#ifndef GeoXGLWidgetH
#define GeoXGLWidgetH
//---------------------------------------------------------------------------
#include <QtOpenGL/QGLWidget>
#include <GL/glu.h>
#include "VisTypes.h"
#include "WalkthroughCameraController.h"
#include "Camera.h"
#include "ViewFrustum.h"
//---------------------------------------------------------------------------



class GeoXGLWidget : public QGLWidget {
   Q_OBJECT
 public:
	 static const int MODE_PICK = 0;
	 static const int MODE_CAM = 1;
	 static const int MODE_CREATE = 2;

 private:
	// datastructures containing the primitives
	std::vector<Point2D> points; 
	std::vector<Line> lines;
	std::vector<Triangle> triangles;
		
	void findClosestPoint(int x, int y);

	// CAMERA, PICKING or CREATE
	int mode;
	// currently selected point
	int pickedPointIndex;
	int lastPickedPointIndex;

	// camera steering
	float zoom;
	float transX, transY;
	int startX, startY;
	Vector3f bgColor;
	
 protected:
	// derived functions
	virtual void paintGL();
	virtual void initializeGL();
	void drawAxis();
	virtual void resizeGL(int width, int height);
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent (QMouseEvent * event);
	virtual void mouseMoveEvent(QMouseEvent *event);

 public:
   GeoXGLWidget(QWidget *parent=0, const QGLWidget *shareWidget=0, Qt::WindowFlags f=0);
   virtual ~GeoXGLWidget();
	
	// add primitives
	int addPoint    (Point2D p);
	int addLine     (Line l);
	int addTriangle (Triangle t);

	// return primitives
	Point2D  getPoint    (int index);
	Line     getLine     (int index);
	Triangle getTriangle	(int index);

	// change primitives
	void setPoint    (int index, Point2D p);
	void setLine     (int index, Line l);
	void setTriangle (int index, Triangle t);
	
	int getNumberOfPoints()    {return points.size();};
	int getNumberOfLines()     {return lines.size();};
	int getNumberOfTriangles() {return triangles.size();};
	
	// clear all primitives
	void clear();
	void clearLines(); // clear lines, leave points and triangles in place
	void clearTriangles(); // clear triangles, leave points and lines in place

	void setCameraMode(){mode = MODE_CAM;};
	void setPickMode()  {mode = MODE_PICK;};
	void setCreateMode(){mode = MODE_CREATE;};
	
	void setBackgroundColor(Vector3f color){bgColor = color;};
	void setBackgroundColor(float red, float green, float blue){bgColor = makeVector3f(red,green,blue);};
	
	// get point which was selcted as last
	int getLastSelectedPointIndex() const { return lastPickedPointIndex; }

 signals:
	void widgetContentChanged();
	void pointSelected(int);	
};

#endif
