#include "stdafx.h"
#include "TriangleRayIntersection.h"


TriangleRayIntersection::TriangleRayIntersection()
{
}


TriangleRayIntersection::~TriangleRayIntersection()
{
}

bool TriangleRayIntersection::getIntersection(Vector3f rayPosition, Vector3f rayDirection, Vector3f pos[3], float &result)
{
	float eps = 0.01;
	float a, f, u, v;
	Vector3f e1 = pos[1] - pos[0];
	Vector3f e2 = pos[2] - pos[0];
	Vector3f h = rayDirection.crossProduct(e2);
	

	a = dot(e1, h);
	if (a > -0.00001 && a < 0.00001)
		return(false);

	f = 1 / a;
	Vector3f s = rayPosition - pos[0];

	u = f * dot(s, h);

	if (u < 0.0 || u > 1.0)
		return(false);

	Vector3f q = s.crossProduct(e1);
	v = f * dot(rayDirection, q);

	if (v < 0.0 || u + v > 1.0)
		return(false);

	// at this stage we can compute t to find out where
	// the intersection point is on the line
	float t = f * dot(e2,q);

	if (t > 0) // ray intersection
	{
		result = t;
		return(true);
	}

	else // this means that there is a line intersection
		// but not a ray intersection
		return (false);

}

float TriangleRayIntersection::dot(Vector3f v1, Vector3f v2)
{
	Vector3f h = v1.componentProduct(v2);
	return h[0] + h[1] + h[2];
}