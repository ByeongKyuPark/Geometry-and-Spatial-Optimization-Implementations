/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Shapes.cpp
Purpose: Implements the functionalities for geometric shape classes such as LineSegments, Rays, Spheres, AABBs, Triangles, Planes, and Frustums. This includes shape construction, transformations, and utilities for geometric computations and debugging.
		  OpenGL Info: 4.6.0 - Build 31.0.101.3413
		  Version: 4.6.0 - Build 31.0.101.3413
		  Vendor: Intel
		  Renderer: Intel(R) UHD Graphics
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
Project: byeonggyu.park_CS350_2
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 2/16/2024
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
	mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
	mStart = start;
	mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
	return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
	mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
	mStart = start;
	mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
	Ray transformedRay;
	transformedRay.mStart = Math::TransformPoint(transform, mStart);
	transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
	return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
	return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
	return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
	mCenter = Vector3::cZero;
	mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
	mCenter = center;
	mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
	/******Student:Assignment2******/
	// The centroid method is roughly describe as: find the centroid (not mean) of all
	// points and then find the furthest away point from the centroid.
	if (points.empty()) {
		mCenter = Vector3();
		mRadius = 0;
		return;
	}

	// Initialize the AABB with the first point to ensure it's valid and can expand
	Aabb boundingBox(points[0], points[0]);

	// Expand the AABB to include all points
	for (const Vector3& point : points) {
		boundingBox.Expand(point);
	}

	// centroid = the center of the Aabb
	mCenter = boundingBox.GetCenter();

	float maxDistanceSquared{};
	for (const Vector3& point : points) {
		float distanceSquared = (mCenter - point).LengthSq();
		if (distanceSquared > maxDistanceSquared) {
			maxDistanceSquared = distanceSquared;
		}
	}
	mRadius = sqrtf(maxDistanceSquared);
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
	/******Student:Assignment2******/
	// The ritter method:
	// Find the largest spread on each axis.
	// Find which axis' pair of points are the furthest (euclidean distance) apart.
	// Choose the center of this line as the sphere center. Now incrementally expand the sphere.
	if (points.empty()) return;

	//init with the first one. (points are not empty)
	Vector3 minX = points[0], maxX = points[0];
	Vector3 minY = points[0], maxY = points[0];
	Vector3 minZ = points[0], maxZ = points[0];

	for (const Vector3& point : points) {
		if (point.x < minX.x) minX = point;
		else if (point.x > maxX.x) maxX = point;

		if (point.y < minY.y) minY = point;
		else if (point.y > maxY.y) maxY = point;

		if (point.z < minZ.z) minZ = point;
		else if (point.z > maxZ.z) maxZ = point;
	}

	float distX = Distance(maxX, minX);
	float distY = Distance(maxY, minY);
	float distZ = Distance(maxZ, minZ);

	Vector3 p1, p2;
	//for ties, pick from x then y, and then z
	if (distX >= distY && distX >= distZ) {
		p1 = minX;
		p2 = maxX;
	}
	else if (distY >= distZ) {
		p1 = minY;
		p2 = maxY;
	}
	else {
		p1 = minZ;
		p2 = maxZ;
	}
	// Initialize the sphere with the center between p1 and p2 and radius as half their distance
	mCenter = (p1 + p2) * 0.5f;
	mRadius = Distance(p1, p2) * 0.5f;
	float radSqrd = mRadius * mRadius;

	// Sphere Expansion
	for (const Vector3& point : points) {
		Vector3 toCenter = mCenter - point;
		float distToPointSqrd = toCenter.LengthSq();
		if (distToPointSqrd > radSqrd) {//point lies outside
			Vector3 b = mCenter + mRadius * toCenter.Normalized();
			float newRadius = (mRadius + sqrtf(distToPointSqrd)) * 0.5f;
			mCenter = (b + point) * 0.5f;
			mRadius = (mRadius + sqrtf(distToPointSqrd)) * 0.5f;//equivalent to "mRadius = Distance(b, point) *0.5f"

			radSqrd = mRadius * mRadius;//update the new rad^2
		}
	}
}


bool Sphere::ContainsPoint(const Vector3& point)
{
	return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
	return mCenter;
}

float Sphere::GetRadius() const
{
	return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
	float posDiff = Math::Length(mCenter - rhs.mCenter);
	float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

	return posDiff < epsilon&& radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
	return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
	//set the aabb to an initial bad value (where the min is smaller than the max)
	mMin.Splat(Math::PositiveMax());
	mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
	mMin = min;
	mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
	return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
	/******Student:Assignment2******/
	// Return the aabb's volume
	Vector3 extents = mMax - mMin;
	return extents.x * extents.y * extents.z;
}

float Aabb::GetSurfaceArea() const
{
	/******Student:Assignment2******/
	// Return the aabb's surface area
	Vector3 extents = mMax - mMin;
	float xyArea = extents.x * extents.y;
	float xzArea = extents.x * extents.z;
	float yzArea = extents.y * extents.z;

	return 2 * (xyArea + xzArea + yzArea);
}

bool Aabb::Contains(const Aabb& aabb) const
{
	/******Student:Assignment2******/
	// Return if aabb is completely contained in this

	// check for x-axis non-containment
	if (mMin.x > aabb.mMin.x || mMax.x < aabb.mMax.x) return false;

	// check for y-axis non-containment
	if (mMin.y > aabb.mMin.y || mMax.y < aabb.mMax.y) return false;

	// check for z-axis non-containment
	if (mMin.z > aabb.mMin.z || mMax.z < aabb.mMax.z) return false;

	// otherwise, then the current AABB entirely contains the passed in AABB
	return true;
}

void Aabb::Expand(const Vector3& point)
{
	for (size_t i = 0; i < 3; ++i)
	{
		mMin[i] = Math::Min(mMin[i], point[i]);
		mMax[i] = Math::Max(mMax[i], point[i]);
	}
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
	Aabb result;
	for (size_t i = 0; i < 3; ++i)
	{
		result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
		result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
	}
	return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
	float pos1Diff = Math::Length(mMin - rhs.mMin);
	float pos2Diff = Math::Length(mMax - rhs.mMax);

	return pos1Diff < epsilon&& pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation)
{
	/******Student:Assignment2******/
	Matrix4 transformationMatrix = BuildTransform(translation, rotation, scale);
	Vector3 center = (mMax + mMin) * 0.5f;
	Vector3 halfExtent = (mMax - mMin) * 0.5f;

	center = TransformPoint(transformationMatrix, center);

	// Making the rotation and scale components of the transformation matrix absolute is necessary as we want the half-extent 
	// to correctly capture the size of the transformed AABB, irrespective of rotation direction. 
	// We only modify the relevant 3x3 portion of the matrix, ignoring translation components, as halfExtent does not need them (it's a vector, where w-component is 0.)
	for (int i{}; i < 3; ++i) {
		for (int j{}; j < 3; ++j) {
			transformationMatrix[i][j] = std::abs(transformationMatrix[i][j]);
		}
	}
	halfExtent = TransformNormal(transformationMatrix, halfExtent);

	mMin = center - halfExtent;
	mMax = center + halfExtent;
}

Vector3 Aabb::GetMin() const
{
	return mMin;
}

Vector3 Aabb::GetMax() const
{
	return mMax;
}

Vector3 Aabb::GetCenter() const
{
	return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
	return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
	return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
	mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
	mPoints[0] = p0;
	mPoints[1] = p1;
	mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
	return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
	mData = Vector4::cZero;
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
	Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
	Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
	/******Student:Assignment1******/
	Vector3 v1 = p1 - p0;
	Vector3 v2 = p2 - p0;

	Vector3 normal = v1.Cross(v2);
	float length = normal.Length();

	// check if the length of the normal is zero (which indicates the points are collinear)
	if (length < std::numeric_limits<float>::epsilon())
	{
		Warn("Plane::Set::Given points are collinear, no unique plane can be defined.");
		return;
	}

	normal.Normalize();

	// Set the plane equation parameters based on the normalized normal vector and a point on the plane.
	// The plane equation is in the form ax + by + cz - d = 0 (<-> ax + by + cz = d) as discussed in class.
	// Here, a, b, and c are the components of the normalized normal vector,
	// and d is the dot product of the normalized normal with a given point on the plane.
	mData.Set(normal.x, normal.y, normal.z, normal.Dot(p0));
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
	Vector3 normalizedNormal = normal.Normalized();

	// Set the plane equation parameters based on the normalized normal vector and a point on the plane.
	// The plane equation is in the form ax + by + cz - d = 0 (<-> ax + by + cz = d) as discussed in class.
	// Here, a, b, and c are the components of the normalized normal vector,
	// and d is the dot product of the normalized normal with a given point on the plane.
	mData.Set(normalizedNormal.x, normalizedNormal.y, normalizedNormal.z, normalizedNormal.Dot(point));
}

Vector3 Plane::GetNormal() const
{
	return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
	return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
	return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
	return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
	const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
	mPoints[0] = lbn;
	mPoints[1] = rbn;
	mPoints[2] = rtn;
	mPoints[3] = ltn;
	mPoints[4] = lbf;
	mPoints[5] = rbf;
	mPoints[6] = rtf;
	mPoints[7] = ltf;

	//left
	mPlanes[0].Set(lbf, ltf, lbn);
	//right
	mPlanes[1].Set(rbn, rtf, rbf);
	//top
	mPlanes[2].Set(ltn, ltf, rtn);
	//bot
	mPlanes[3].Set(rbn, lbf, lbn);
	//near
	mPlanes[4].Set(lbn, ltn, rbn);
	//far
	mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
	return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
	return gDebugDrawer->DrawFrustum(*this);
}
