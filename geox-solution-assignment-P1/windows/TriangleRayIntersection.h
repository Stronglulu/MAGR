#pragma once
#include "LinearAlgebra.hpp"

class Ray;
class TriangleRayIntersection
{
public:
	TriangleRayIntersection();
	~TriangleRayIntersection();
	bool getIntersection(Vector3f rayPosition, Vector3f rayDirection, Vector3f triangle[3], float &result);


private:
	float dot(Vector3f v1, Vector3f v2);
};