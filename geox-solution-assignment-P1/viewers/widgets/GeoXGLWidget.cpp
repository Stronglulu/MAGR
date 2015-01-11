//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "GeoXOutput.h"
#include "GeoXGLWidget.h"
#include <QtGui/qevent.h>
//---------------------------------------------------------------------------


GeoXGLWidget::GeoXGLWidget(QWidget *parent, const QGLWidget *shareWidget, Qt::WindowFlags f)
: QGLWidget(parent, shareWidget, f)
{
	initializeGL();
	mode = MODE_CAM;
	pickedPointIndex = -1;
	lastPickedPointIndex = -1;
	zoom = 1.0f;
	transX = transY = 0;
	bgColor = makeVector3f(0.1f,0.1f,0.2f);
	this->clear();
}

void GeoXGLWidget::initializeGL() 
{
	glViewport(0,0,width(),height());
}

void GeoXGLWidget::clear()
{
	points.clear();
	lines.clear();
	triangles.clear();
	updateGL();
}

void GeoXGLWidget::clearLines()
{
	lines.clear();
	updateGL();
}

void GeoXGLWidget::clearTriangles()
{
	triangles.clear();
	updateGL();
}

void GeoXGLWidget::drawAxis()
{
		glLineWidth(1.0f);
		glColor4f(1.0f,0.0f,0.0f,1.0f);
		glBegin(GL_LINES);
		glVertex3f(-2000.0f, 0.0f, 0.0f);
		glVertex3f( 2000.0f, 0.0f, 0.0f);
		glEnd();
		glColor4f(0.0f,1.0f,0.0f,1.0f);
		glBegin(GL_LINES);
		glVertex3f(0.0f, -2000.0f, 0.0f);
		glVertex3f(0.0f,  2000.0f, 0.0f);
		glEnd();
}

void GeoXGLWidget::paintGL()
{
	glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
	gluOrtho2D(-10.0f*zoom,10.0f*zoom,(-10.0f*zoom)/((float)width()/height()),(10.0f*zoom)/((float)width()/height()));
	
	glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glTranslatef(transX, transY, 0);
	glClearColor(bgColor[0],bgColor[1],bgColor[2],0.0f);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	drawAxis();

	for(unsigned int i = 0;i<points.size();i++)
	{
		glPointSize(points[i].size);
		glColor4f(points[i].color[0],points[i].color[1],points[i].color[2],points[i].color[3]);
		glBegin(GL_POINTS);
			glVertex2f(	points[i].position[0],
							points[i].position[1]);
		glEnd();
	}

	// draw selected point
	if (lastPickedPointIndex > -1 && lastPickedPointIndex < (int)points.size())
	{
		glPushAttrib(GL_ENABLE_BIT); 
		glLineStipple(1, 0x0707);
		glEnable(GL_LINE_STIPPLE);

		glColor4f(points[lastPickedPointIndex].color[0],points[lastPickedPointIndex].color[1],points[lastPickedPointIndex].color[2],points[lastPickedPointIndex].color[3]);

		glBegin(GL_LINE_STRIP);

		float px = points[lastPickedPointIndex].position[0];
		float py = points[lastPickedPointIndex].position[1];
		float ampl = 0.5f;

		for (unsigned i=0; i < 33; i++)
		{
			float alpha = 2.0f * M_PI * float(i) / float(32);
			glVertex2f(ampl * cos(alpha) + px, ampl * sin(alpha) + py);
		}

		glEnd();
		glPopAttrib();
	}

	for(unsigned int j = 0;j<lines.size();j++)
	{
	   glLineWidth(lines[j].thickness);
		glColor4f(lines[j].color[0],lines[j].color[1],lines[j].color[2],lines[j].color[3]);
		glBegin(GL_LINES);
			glVertex2f(	points[lines[j].vertices[0]].position[0],
							points[lines[j].vertices[0]].position[1]);
			glVertex2f(	points[lines[j].vertices[1]].position[0],
							points[lines[j].vertices[1]].position[1]);
		glEnd();
	}

	for(unsigned int k = 0;k<triangles.size();k++)
	{
		glColor4f(triangles[k].color[0],triangles[k].color[1],triangles[k].color[2],triangles[k].color[3]);
		glBegin(GL_TRIANGLES);
			glVertex2f(	points[triangles[k].vertices[0]].position[0],
							points[triangles[k].vertices[0]].position[1]);
			glVertex2f(	points[triangles[k].vertices[1]].position[0],
							points[triangles[k].vertices[1]].position[1]);
			glVertex2f( points[triangles[k].vertices[2]].position[0],
							points[triangles[k].vertices[2]].position[1]);
		glEnd();
	}
}

int GeoXGLWidget::addPoint(Point2D p)
{
	points.push_back(p);
	return points.size()-1;
}

int GeoXGLWidget::addLine(Line l)
{
	lines.push_back(l);
	return lines.size()-1;
}

int GeoXGLWidget::addTriangle(Triangle t)
{
	triangles.push_back(t);
	return triangles.size()-1;
}

Point2D GeoXGLWidget::getPoint(int index)
{
	if ((index < 0) || (index > ((int)points.size()-1)))
	{
		output << "WARNING: No Point entry at index " << index << " ! A default value is returned!\n";
		return Point2D();
	}
	return points[index];
}

Line GeoXGLWidget::getLine(int index)
{
	if ((index < 0) || (index > ((int)lines.size()-1)))
	{
		output << "WARNING: No Line entry at index " << index << " ! A default value is returned!\n";
		return Line();
	}
	return lines[index];
}

Triangle GeoXGLWidget::getTriangle(int index)
{
	if ((index < 0) || (index > ((int)triangles.size()-1)))
	{
		output << "WARNING: No Triangle entry at index " << index << " ! A default value is returned!\n";
		return Triangle();
	}
	return triangles[index];
}

void GeoXGLWidget::setPoint(int index, Point2D p)
{
	if ((index < 0) || (index > ((int)points.size()-1)))
	{
		output << "WARNING: Index " << index << " out of bounds!\n";
		output << "WARNING: No changes applied!\n";
		return;
	}
	points[index] = p;
}

void GeoXGLWidget::setLine(int index, Line l)
{
	if ((index < 0) || (index > ((int)lines.size()-1)))
	{
		output << "WARNING: Index " << index << " out of bounds!\n";
		output << "WARNING: No changes applied!\n";
		return;
	}
	lines[index] = l;
}

void GeoXGLWidget::setTriangle(int index, Triangle t)
{
	if ((index < 0) || (index > ((int)triangles.size()-1)))
	{
		output << "WARNING: Index " << index << " out of bounds!\n";
		output << "WARNING: No changes applied!\n";
			return;
	}
	triangles[index] = t;
}

GeoXGLWidget::~GeoXGLWidget()
{
}

void GeoXGLWidget::mousePressEvent(QMouseEvent *event)
{
	if (mode == MODE_PICK)
	{
		findClosestPoint(event->x(),event->y());
		if (pickedPointIndex != -1)
			emit pointSelected(pickedPointIndex);
	}
	if (mode == MODE_CAM)
	{
		startX = event->x();
		startY = event->y();
	}
	if(mode == MODE_CREATE)
	{
		startX = event->x();
		startY = event->y();
		Point2D pNew;
		int newIndex = this->addPoint(pNew);
		double modelview[16], projection[16];
		int viewport[4];
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetIntegerv( GL_VIEWPORT, viewport );
		Vector3d objPos;
		gluUnProject(event->x(), viewport[3]-event->y(), 0.0, modelview, projection, viewport, &objPos[0], &objPos[1], &objPos[2] );
		points[newIndex].position = makeVector2f(objPos[0], objPos[1]);
	}
	updateGL();
}

void GeoXGLWidget::mouseReleaseEvent (QMouseEvent * event)
{
	if((mode == MODE_PICK) && (pickedPointIndex != -1))
	{
		double modelview[16], projection[16];
		int viewport[4];
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetIntegerv( GL_VIEWPORT, viewport );
		Vector3d objPos;
		gluUnProject(event->x(), viewport[3]-event->y(), 0.0, modelview, projection, viewport, &objPos[0], &objPos[1], &objPos[2] );
		points[pickedPointIndex].position = makeVector2f(objPos[0], objPos[1]);
		pickedPointIndex = -1;
		emit widgetContentChanged();
	}
	if (mode == MODE_CAM)
	{
		// nothing to do here
	}
	updateGL();
}

void GeoXGLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if(mode == MODE_CAM)
	{
		if(event->buttons() & Qt::RightButton)
		{	
			int diffx = event->x() - startX;
			int diffy = event->y() - startY;
			if (abs(diffx) > abs(diffy))
				zoom *= 1 + diffx * 0.01;
			else
				zoom *= 1 + diffy * 0.01;
		}
		if(event->buttons() & Qt::LeftButton)
		{	
			int diffx = event->x() - startX;
			int diffy = event->y() - startY;
			transX += diffx * 0.03 * zoom;			
			transY -= diffy * 0.03 * zoom;			
		}
	}
	if((mode == MODE_PICK) && (pickedPointIndex != -1))
	{
		double modelview[16], projection[16];
		int viewport[4];
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetIntegerv( GL_VIEWPORT, viewport );
		Vector3d objPos;
		gluUnProject(event->x(), viewport[3]-event->y(), 0.0, modelview, projection, viewport, &objPos[0], &objPos[1], &objPos[2] );
		points[pickedPointIndex].position = makeVector2f(objPos[0], objPos[1]);
	}
	if(mode == MODE_CREATE)
	{
		Point2D pNew;
		int newIndex = this->addPoint(pNew);
		double modelview[16], projection[16];
		int viewport[4];
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetIntegerv( GL_VIEWPORT, viewport );
		Vector3d objPos;
		gluUnProject(event->x(), viewport[3]-event->y(), 0.0, modelview, projection, viewport, &objPos[0], &objPos[1], &objPos[2] );
		points[newIndex].position = makeVector2f(objPos[0], objPos[1]);
	}

	startX = event->x();
	startY = event->y();
	updateGL();
}

void GeoXGLWidget::resizeGL(int width, int height)
{
	this->resize(width,height);
	glViewport(0,0,this->width(),this->height());
}

void GeoXGLWidget::findClosestPoint(int x, int y)
{
	int ind = -1;
	double modelview[16], projection[16];
	int viewport[4];
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetIntegerv( GL_VIEWPORT, viewport );
	double winX, winY, winZ;
	for(int i = 0; i < (int)points.size(); i++)
	{
		gluProject(points[i].position[0],points[i].position[1],0.0,
					  modelview,projection,viewport,&winX,&winY,&winZ);
		if((makeVector2f(winX,viewport[3]-winY)-makeVector2f(x,y)).getSqrNorm() < 16.0f)
		{
			if(points[i].canBeModified)
			{
				pickedPointIndex = i;
				lastPickedPointIndex = i;
				break;
			}
		}
	}
}

#ifdef _WIN32
#include "moc/GeoXGLWidget_moc.cpp"
#else
#include "moc_GeoXGLWidget.cpp"
#endif

