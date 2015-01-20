//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "MyMeshExperiment.h"
//---------------------------------------------------------------------------
#include "Properties.h"
#include "GLGeometryViewer3D.h"
#include "GeoXOutput.h"   
#include "MeshImporterSMFOBJ.h"
#include "SimpleGLMeshMaterial.h"
#include "TriangleMesh.h"
#include "ObjectViewsTable.h"
//---------------------------------------------------------------------------
#include "GeoXOutput.h"                         // <--- this include lets you output text in the main window
#include "Camera.h"
#include "TriangleRayIntersection.h"

IMPLEMENT_GEOX_CLASS(MyMeshExperiment, 0)
{
	BEGIN_CLASS_INIT(MyMeshExperiment);
	ADD_OBJECT_PROP(mesh, 0, TriangleMesh::getClass(), true)
		ADD_OBJECT_PROP(renderer, 0, SimpleGLMeshMaterial::getClass(), true)
		ADD_NOARGS_METHOD(MyMeshExperiment::importMesh)
		ADD_NOARGS_METHOD(MyMeshExperiment::calculateDot)
		ADD_NOARGS_METHOD(MyMeshExperiment::calculateMatrixNormal)
		ADD_NOARGS_METHOD(MyMeshExperiment::getViewerInfo)             // <--- get viewer info

	ADD_SEPARATOR("Vectors & Matrices")          // 
		ADD_INT32_PROP(gridSize, 0)
		ADD_VECTOR3F_PROP(incomingRay, 0)                // <---
		ADD_VECTOR3F_PROP(vertex1, 0)                // <---
		ADD_VECTOR3F_PROP(vertex2, 0)                // <---
		ADD_VECTOR3F_PROP(vertex3, 0)                // <---
}

QWidget *MyMeshExperiment::createViewer() {
	viewer = new BasicGLViewer3D();
	viewer->addRenderCallback(this);
	return viewer;
}

MyMeshExperiment::MyMeshExperiment()
{
	viewer = NULL;
	mesh = NULL;
	renderer = new SimpleGLMeshMaterial();

	gridSize = -1;
	incomingRay = makeVector3f(0, 1, -1);
	vertex1 = makeVector3f(0, 0, 0);
	vertex2 = makeVector3f(0, 1, 0);
	vertex3 = makeVector3f(1, 0, 0);
}

MyMeshExperiment::~MyMeshExperiment()
{
	delete renderer;
}

void MyMeshExperiment::importMesh()
{

	QString filename = QFileDialog::getOpenFileName(viewer, "Open OBJ File...", QString(), "3D Objects (*.obj *.smf)", 0, 0);
	if (filename != "") {
		delete mesh;
		mesh = NULL;
		MeshImporterSMFOBJ importer;
		importer.performImport(qString2STLString(filename));
		mesh = importer.createTriangleMesh();
	}

	// display changes
	ObjectViewsTable::update(this);
	viewer->refresh();
}

void MyMeshExperiment::renderGL()
{
	if (renderer && mesh) {
		renderer->draw(mesh);
	}
}

void MyMeshExperiment::getViewerInfo() 
{
	//// Clear image
	//for (int x = 0; x < size; ++x)
	//	for (int y = 0; y < size; ++x)
	//		colours[x][y] = makeVector3f(1, 1, 1);

	controller = viewer->getController();
	Vector3f tris[3] = {vertex1, vertex2, vertex3};

	output << "\n (x, y, z)" << "\n";
	output << "vertex1: " << tris[0] << "\n"
		<< "vertex2: " << tris[1] << "\n"
		<< "vertex3: " << tris[2] << "\n";

	Vector3f outgoing;
	getOutgoingReflection(incomingRay, tris, outgoing);

	output << "incoming: " << incomingRay << "\n";
	output << "outgoing: " << outgoing << "\n";

	getRays();
	shootRays();
	saveImage();
	output << "\ndone\n";
}

void MyMeshExperiment::getRays()
{	
	int halfsize = size / 2;
	controller = viewer->getController();
	Vector3f lookat, position, right, up, view; 
	lookat = controller->getCamera()->getLookAt();
	position = controller->getCamera()->getPosition();
	view = lookat - position;
	up = controller->getCamera()->getUp();
	right = up.crossProduct(view);
	up.normalize();
	right.normalize();
	float distance = controller->getDistance();

	for (int dx = -halfsize; dx <= halfsize; dx++)
		for (int dy = -halfsize; dy <= halfsize; dy++)
		{
			Vector3f dr = right * distance * dx / halfsize;
			Vector3f du = up * distance * dy / halfsize;
			Vector3f pixelpos = lookat - dr - du;
			rays[dx + halfsize][dy + halfsize] = tuple<Vector3f, Vector3f>(position, pixelpos - position);

		}
}

void MyMeshExperiment::shootRays()
{
	TriangleRayIntersection* tri = new TriangleRayIntersection();

	if (mesh == NULL) return;
	DynamicArrayOfStructures *pts = mesh->getVertices();
	if (!pts->providesAttribute("position")) return;
	AAT POS = pts->getAAT("position");

	AAT COL;
	bool hasCol = pts->providesAttribute("color");
	if (hasCol) COL = pts->getAAT("color");

	DynamicArrayOfStructures *idx = mesh->getTriangles();
	if (!idx->providesAttribute("index")) return;
	AAT IDX = idx->getAAT("index");

	bool reflectionDepth = true;


	// loop over rays
	for (int x = 0; x <= size; x++)
	{
		for (int y = 0; y <= size; y++)
		{

			tuple<Vector3f, Vector3f> ray = rays[x][y];				// 0: ray origin 1: ray direction
			float distance;											// <-- scalar multiplied with ray direction is distance
			float minDistance = 99999;								// <-- used to find nearest triangle
			int minDistanceId = -1;									// <-- check for background
			Vector3f bestColour, objectColour, shadowColour;
			Vector3f outgoingRay;
			bool thereWasARefelction = false;
			
			bool thereIsAReflection = false;
			// loop over triangle
			const card32 numTri = idx->getNumEntries();
			for (card32 i = 0; i < numTri; i++) 
			{
				Vector3i tind = idx->get<int32, 3>(i, IDX);			// <-- triangle index of vertices (comes from triangle dynamic thing)
				Vector3f triangle[3];								// <-- position of all vertices of the triangle
				triangle[0] = pts->get<float32, 3>(tind[0], POS);
				triangle[1] = pts->get<float32, 3>(tind[1], POS);
				triangle[2] = pts->get<float32, 3>(tind[2], POS);


				// get intersection + distance
				bool xt = tri->getIntersection(std::get<0>(ray), std::get<1>(ray), triangle, distance);		// <-- distance is a result
				if (xt)
				{
					if (distance < minDistance)						// <-- check if closest
					{
						getOutgoingReflection(std::get<1>(ray), triangle, outgoingRay); // get outgoing ray for reflection
						reflectRays[x][y] = outgoingRay;
						minDistance = distance;							// <-- set current triangle as closest					
						minDistances[x][y] = distance;
						minDistanceId = i;								// <-- a check to see if current current triangle is closer than existing

						// calculations for local shading
						Vector3f hit = (std::get<0>(ray) +std::get<1>(ray)*distance);				// <-- location of where ray hit triangle 
						Vector3f light = lightPos - hit;								// <-- intersection to lightsource vector
						Vector3f normal = makeVector3f(0,0,0);
						calculateSurfaceNormal(triangle, normal);
						light.normalize();
						normal.normalize();
						normal = makeVector3f(0, 0, 0) - normal;
						float weight = std::abs(light * normal);


						// Colours.
						if (hit[1] < 0) //plane, chekered pattern (red/white)
						{
							if (mod(hit[0] / 20, 2) == mod(hit[2] / 20, 2))
								objectColour = makeVector3f(1, 0, 0);							
							else
								objectColour = makeVector3f(1, 1, 1);							
						}
						else // object colours.
							objectColour = makeVector3f(1, 1, 1);
						

						// Calculate shadows.
						shadowColour =  calculateShadow(ray, distance, weight);					
						colours[x][y] = (objectColour + shadowColour) / 2; //<-- Object colour, local shading and shadow
					}
					
				}				
			}
			

			// Calculate Reflection
			float distance2;											// <-- scalar multiplied with ray direction is distance
			float minDistance2 = 99999;								// <-- used to find nearest triangle
			int minDistanceId2 = -1;									// <-- check for background
			Vector3f bestColour2, shadowColour2;
			Vector3f closestHit = std::get<0>(ray) +std::get<1>(ray)*minDistance;

			// Loop over all triangles and check for intersection with reflected ray.
			for (card32 j = 0; j < numTri; j++)
			{
				Vector3i tind2 = idx->get<int32, 3>(j, IDX);			// <-- triangle index of vertices (comes from triangle dynamic thing)
				Vector3f pos2[3];									// <-- position of all vertices of the triangle
				pos2[0] = pts->get<float32, 3>(tind2[0], POS);
				pos2[1] = pts->get<float32, 3>(tind2[1], POS);
				pos2[2] = pts->get<float32, 3>(tind2[2], POS);


				// get intersection + distance
				
				if (closestHit[1] > 0)								// <-- everything above plane is reflective
				{
					thereIsAReflection = tri->getIntersection(closestHit + outgoingRay*0.02, outgoingRay, pos2, distance2);		// <-- distance is a result
					if (thereIsAReflection)
					{
						if (distance2 < minDistance2)						// <-- check if closest
						{
							if (perfectReflection)
							{
								minDistance2 = distance2;							// <-- set current triangle as closest
								minDistanceId2 = j;								// <-- a check to see if current current triangle is closer than existing
								Vector3f newHit = closestHit + outgoingRay*distance2;
								tuple<Vector3f, Vector3f> ray2 = make_tuple(closestHit, outgoingRay);
								Vector3f light = lightPos - newHit;								// <-- intersection to lightsource vector
								Vector3f normal = makeVector3f(0, 0, 0);
								calculateSurfaceNormal(pos2, normal);
								light.normalize();
								normal.normalize();
								normal = makeVector3f(0, 0, 0) - normal;
								float weight = std::abs(light * normal);

								if (newHit[1] <= 1)	//<-- plane reflection
								{
									// Chekered pattern
									if (mod(newHit[0] / 20, 2) == mod(newHit[2] / 20, 2))
										bestColour2 = makeVector3f(1, 0, 0);
									else
										bestColour2 = makeVector3f(1, 1, 1);
								}

								shadowColour2 = calculateShadow(ray2, distance2, weight);
								colours[x][y] = (colours[x][y] + shadowColour2 + bestColour2) / 3;		//<-- perfect reflection, added objectcolour and shadow colour of the reflected area 
							}
							else
							{
								// Calculate new basis u, v, w 
								Vector3f w = outgoingRay;
								w.normalize();
								Vector3f t = w;
								int smallestCompo = 0;

								for (int i = 1; i < 3; ++i)
									if (t[i] < t[smallestCompo])
										smallestCompo = i;

								t[smallestCompo] = 0;

								Vector3f u = t.crossProduct(w);
								Vector3f v = u.crossProduct(w);
								float glossyness = 40;
								float numReflected = 0;
								float color = 0;
								Vector3f totalColor = makeVector3f(0, 0, 0);

								for (int j = 0; j < 18; ++j)
								{
									float rx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
									float randomX = -glossyness / 2 + rx * glossyness;
									float ry = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
									float randomY = -glossyness / 2 + ry * glossyness;

									Vector3f newOutgoingRay = outgoingRay + u.operator*(randomX)+v.operator*(randomY);

									thereIsAReflection = tri->getIntersection(closestHit + newOutgoingRay*0.02, newOutgoingRay, pos2, distance2);
									{
										numReflected++;
										Vector3f newHit = closestHit + newOutgoingRay*distance2;
										tuple<Vector3f, Vector3f> ray2 = make_tuple(closestHit, outgoingRay);
										Vector3f light = lightPos - newHit;								// <-- intersection to lightsource vector
										Vector3f normal = makeVector3f(0, 0, 0);
										calculateSurfaceNormal(pos2, normal);
										light.normalize();
										normal.normalize();
										normal = makeVector3f(0, 0, 0) - normal;
										float weight = std::abs(light * normal);

										if (newHit[1] <= 1)	//<-- plane reflection
										{
											// Chekered pattern
											if (mod(newHit[0] / 20, 2) == mod(newHit[2] / 20, 2))
												bestColour2 = makeVector3f(1, 0, 0);
											else
												bestColour2 = makeVector3f(1, 1, 1);
										}

										shadowColour2 = calculateShadow(ray2, distance2, weight);
										totalColor += (shadowColour2 + bestColour2) / 2;
									}
								}
								colours[x][y] =(colours[x][y] + totalColor / numReflected)/2;		//<-- perfect reflection, added objectcolour and shadow colour of the reflected area 

							}
						}
					}
				}
			}			
		}//rays
	}
	delete tri;
}

Vector3f MyMeshExperiment::calculateShadow(tuple<Vector3f, Vector3f> ray, float distance, float weight)
{
	Vector3f bestColour;

	// Calculate shadows, including local shading.
	if (softShadows)
	{
		float shadowWeight = getSoftShadow(ray, distance, lightPos);

		if (shadowWeight == 1)	//<-- no shadow
			bestColour = makeVector3f((weight + 0.1f) / 1.1f, (weight + 0.1f) / 1.1f, (weight + 0.1f) / 1.1f);
		else
			bestColour = makeVector3f((weight + 0.1f) / 1.1f  * shadowWeight, (weight + 0.1f) / 1.1f * shadowWeight, (weight + 0.1f) / 1.1f * shadowWeight);		// <-- default object color (almost black)

	}
	else //hard shadows
	{
		bool shadow = checkShadow(ray, distance, lightPos);		// <-- check if there are triangles blocking the light
		if (!shadow)
			bestColour = makeVector3f((weight + 0.1f) / 1.1f, (weight + 0.1f) / 1.1f, (weight + 0.1f) / 1.1f);
		else
			bestColour = makeVector3f(0.1, 0.1, 0.1);		// <-- default shadow color (almost black)
	}
	return bestColour; //<-- Object colour, local shading and shadow
}


void MyMeshExperiment::calculateDot()
{
	output << "dot: " << incomingRay * vertex1 << "\n";
}

void MyMeshExperiment::calculateMatrixNormal()
{
	Vector3f normal;
	Vector3f tris[3] = { vertex1, vertex2, vertex3 };

	//tris[0] = triangleRefl * makeVector3f(1, 0, 0);
	//tris[1] = triangleRefl * makeVector3f(0, 1, 0);
	//tris[2] = triangleRefl * makeVector3f(0, 0, 1);

	calculateSurfaceNormal(tris, normal);

	output << "SurfaceNormal: " << normal << "\n";
}

void MyMeshExperiment::calculateSurfaceNormal(Vector3f triangle[3], Vector3f &normal)
{
	Vector3f n = ((triangle[1] - triangle[0]).crossProduct(triangle[2] - triangle[0]));
	float lengthNorm = norm(n);
	if (lengthNorm > 0)
		normal = n / lengthNorm;
	else
	normal = n;
}

void MyMeshExperiment::getOutgoingReflection(Vector3f incomingRay, Vector3f triangle[3], Vector3f &outgoingRay)
{
	Vector3f normal;
	calculateSurfaceNormal(triangle, normal);

	Vector3f ri = incomingRay;
	Vector3f d = ri.componentProduct(normal);
	float dotProd = d[0] + d[1] + d[2];
	//outgoingRay = ri - (normal.operator*(2)).operator*(dotProd);
	outgoingRay = ri - normal*((normal*ri)*2.0f);
}

float MyMeshExperiment::getSoftShadow(tuple<Vector3f, Vector3f> ray, float distance, Vector3f areaLightPos)
{
	Vector3f hit = (std::get<0>(ray) +std::get<1>(ray)*distance);				// <-- location of where ray hit triangle 
	Vector3f light = areaLightPos - hit;								// <-- intersection to lightsource vector
	Vector3f newUp, newRight, newLightPos, lightPointx, lightPointz;
	float areaLightW = 10;
	float areaLightH = 10;
	float halfW = areaLightW / 2;
	float halfH = areaLightH / 2;
	int gridSize = 5;	// <-- kxk grid
	int numShadowRays = gridSize * gridSize;
	float rndR, rndU, newDistance;
	TriangleRayIntersection* tri = new TriangleRayIntersection();
	int numOfShadows = 0;
	float shadowIntensity = 0.5;	//<-- Avoid pitch black shadow.
	float lightGridSpacer = (gridSize - 1) / 2;

	DynamicArrayOfStructures *idx = mesh->getTriangles();
	if (!idx->providesAttribute("index"));
	AAT IDX = idx->getAAT("index");

	if (mesh == NULL);
	DynamicArrayOfStructures *pts = mesh->getVertices();
	if (!pts->providesAttribute("position"));
	AAT POS = pts->getAAT("position");



		lightPointx = makeVector3f(areaLightW / gridSize, 0, 0);			// <-- get the distance between the center of 2 subgrids of the light area
		lightPointz = makeVector3f(0, 0, areaLightH / gridSize);

		// Loop over subgrid of the area light
		for (int x = -lightGridSpacer; x <= lightGridSpacer; ++x)
			for (int z = -lightGridSpacer; z <= lightGridSpacer; ++z)
			{
				newLightPos = areaLightPos + lightPointx.operator*(x) + lightPointz.operator*(z);
				light = newLightPos - hit;									// <-- vector from the hit position and the lightposition of subgrid

				// loop over triangles
				const card32 numTri = idx->getNumEntries();
				for (card32 i = 0; i < numTri; i++)
				{
					Vector3i tind = idx->get<int32, 3>(i, IDX);
					Vector3f pos[3];
					pos[0] = pts->get<float32, 3>(tind[0], POS);
					pos[1] = pts->get<float32, 3>(tind[1], POS);
					pos[2] = pts->get<float32, 3>(tind[2], POS);

					bool intersect = tri->getIntersection(hit + light*0.01, light, pos, newDistance);				// <-- hit+light*0.01 prevents shadow dots 
					if (intersect && newDistance <= 1)
					{
						++numOfShadows;										// <-- Count number of shadow rays casting a shadow
						break;												// <-- If at least one triangle causes a shadow, 
																			//	   there is shadow, stop to look in all the triangles.
					}
				}	
			}

	return 1 - ((float)numOfShadows / (float)numShadowRays * shadowIntensity); // <-- the lower the number, the darker it will get
}

bool MyMeshExperiment::checkShadow(tuple<Vector3f, Vector3f> ray, float distance,Vector3f lightPos)
{
	Vector3f hit = (std::get<0>(ray) + std::get<1>(ray)*distance);				// <-- location of where ray hit triangle 
	Vector3f light = lightPos - hit;								// <-- intersection to lightsource vector
	float newDistance;
	TriangleRayIntersection* tri = new TriangleRayIntersection();

	DynamicArrayOfStructures *idx = mesh->getTriangles();
	if (!idx->providesAttribute("index"));
	AAT IDX = idx->getAAT("index");

	if (mesh == NULL);
	DynamicArrayOfStructures *pts = mesh->getVertices();
	if (!pts->providesAttribute("position"));
	AAT POS = pts->getAAT("position");

	// loop over triangle
	const card32 numTri = idx->getNumEntries();
	for (card32 i = 0; i < numTri; i++) 
	{
		Vector3i tind = idx->get<int32, 3>(i, IDX);
		Vector3f pos[3];
		pos[0] = pts->get<float32, 3>(tind[0], POS);
		pos[1] = pts->get<float32, 3>(tind[1], POS);
		pos[2] = pts->get<float32, 3>(tind[2], POS);

		bool intersect = tri->getIntersection(hit+light*0.01, light, pos, newDistance);				// <-- hit+light*0.01 prevents shadow dots 
		if (intersect && newDistance <= 1)		// && newDistance > 0.01)
			return true;		//There is shadow.
	}
	return false;
}

int MyMeshExperiment::mod(float f, int i)
{
	int x;
	if (f >= 0)
		x = (int)f%i;
	else
		x = 1 - (((int)f%i + i) % i);

	return x;
}

void MyMeshExperiment::saveImage()
{
	QImage image(size + 1, size+1, QImage::Format_RGB32);
	for (int x = 0; x <= size; x++)
	{
		for (int y = 0; y <= size; y++)
		{
			image.setPixel(x, y, qRgb(int(colours[x][y][0] * 255), int(colours[x][y][1] * 255), int(colours[x][y][2] * 255)));
		}
	}
	image.save(QString("test.png"));
}
