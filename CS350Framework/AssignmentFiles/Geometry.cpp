/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Geometry.cpp
Purpose: Implementation for various geometric intersection and containment tests.
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
		  OpenGL Info: 4.6.0 - Build 31.0.101.3413
		  Version: 4.6.0 - Build 31.0.101.3413
		  Vendor: Intel
		  Renderer: Intel(R) UHD Graphics
Project: byeonggyu.park_CS350_2
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 1/14/2024
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"


// Helper function to compute the signed distance between a point and a plane.
// Parameters:
//   point - A 3D point represented by its x, y, and z coordinates.
//   plane - A 4D vector representing the plane. The first three components (x, y, z) 
//           are the normal vector of the plane, and the fourth component (w) is the 
//           signed distance from the origin to the plane along its normal vector.
// Returns:
//   The signed distance from the point to the plane.
float CalcSignedDistanceFromPointToPlane(const Vector3& point, const Vector4& plane) {
	// Calculate the signed distance from the point to the plane using the plane equation.
	// The plane equation is ax + by + cz - d = 0, where (a, b, c) is the normal vector of the plane,
	// (x, y, z) are the coordinates of the point, and d is the signed distance from the origin
	// to the plane along its normal vector. The signed distance indicates on which side of the 
	// plane the point lies and how far it is from the plane. A positive distance means the point 
	// is on the side of the plane to which the normal vector points, while a negative distance 
	// means it's on the opposite side.

	return point.x * plane.x + point.y * plane.y + point.z * plane.z - plane.w;
}

// Helper function to checks if a line segment (edge) intersects with a plane.
// Parameters:
//   edgeStart - Start point of the edge in 3D space.
//   edgeEnd - End point of the edge in 3D space.
//   plane - Plane represented by a 4D vector (normal and distance from origin).
//   t - Intersection parameter along the edge where the intersection occurs.
//   epsilon - Small threshold for numerical precision.
// Returns:
//   True if the edge intersects the plane within the segment; false otherwise.
bool EdgeIntersectsPlane(const Vector3& edgeStart, const Vector3& edgeEnd, const Vector4& plane, float& t, float epsilon) {
	Vector3 rayDir = edgeEnd - edgeStart;
	return RayPlane(edgeStart, rayDir, plane, t, epsilon) && t <= 1.f;
}

// Projects a point onto a plane by computing the signed distance from the point to the plane and moving the point along the plane's normal.
Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance)
{
	/******Student:Assignment1******/
	// The plane equation is ax + by + cz - d = 0.
	// Here, 'planeDistance' is equivalent to d in the plane equation.
	// 'pointDistanceFromPlane' calculates the signed distance from 'point' to the plane along the direction of the normal.
	float pointDistanceFromPlane = point.Dot(normal) - planeDistance;

	Vector3 projectedPoint = point - normal * pointDistanceFromPlane;

	return projectedPoint;
}

// Calculates barycentric coordinates for a point relative to a line segment, handling degenerate cases and expanding the line segment range based on epsilon.
bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
	float& u, float& v, float epsilon)
{
	/******Student:Assignment1******/
	Vector3 ab = a - b;
	float abLengthSq = ab.LengthSq();
	if (abLengthSq < std::numeric_limits<float>::epsilon()) {
		u = v = 0.f;
		return false;
	}
	Vector3 pb = point - b;
	u = pb.Dot(ab) / abLengthSq;
	v = 1.f - u;

	// Check if point is not within the expanded line segment range
	return !(v < -epsilon || v > 1.f + epsilon);
}

// Calculates barycentric coordinates for a point relative to a triangle using Cramer's rule and checks containment within an epsilon-expanded triangle.
bool BarycentricCoordinates(const Vector3& point, const Vector3& A, const Vector3& B, const Vector3& C,
	float& u, float& v, float& w, float epsilon)
{
	/******Student:Assignment1******/
	Vector3 v0 = point - C;
	Vector3 v1 = A - C;
	Vector3 v2 = B - C;

	//e = ua+vb
	//f = uc+vd
	float e = v0.Dot(v1);
	float a = v1.LengthSq();
	float b = v1.Dot(v2);
	float f = v0.Dot(v2);
	float c = b;
	float d = v2.LengthSq();

	float denom = a * d - b*c;
	// Check degeneracy
	if (std::abs(denom) < std::numeric_limits<float>::epsilon()) {
		u = v = w = 0.f;
		return false;
	}

	// Cramer's rule
	u = (e * d - b * f)/denom;
	v = (a * f - e * c)/denom;
	w = 1.f - (u + v);

	// check ranges (non-containment)
	return !(u < -epsilon || u > 1.f + epsilon ||
			v < -epsilon || v > 1.f + epsilon ||
			w < -epsilon || w > 1.f + epsilon);
}

// Classifies a point's position relative to a plane by comparing the signed distance to an epsilon threshold.
IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
	/******Student:Assignment1******/
	// This function determines the relationship between a point and a plane based on the signed distance.
	// A positive signed distance is treated as "inside" (i.e., on the same side as the plane's normal).
	// A negative signed distance indicates the point is on the opposite side (considered "outside").
	// If the signed distance is within 'epsilon', the point is coplanar with the plane.
	float signedDistance = CalcSignedDistanceFromPointToPlane(point, plane);

	if (std::abs(signedDistance) <= epsilon) {
		return IntersectionType::Coplanar;
	}
	else if (signedDistance > 0) {
		return IntersectionType::Inside;
	}
	else {
		return IntersectionType::Outside;
	}
}

// Determines if a point is inside a sphere by comparing the squared distance to the sphere's center with the squared radius.
bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
	///******Student:Assignment1******/
	Vector3 point2SphereCenter = sphereCenter - point;
	float squaredDistance = point2SphereCenter.LengthSq();

	return squaredDistance <= sphereRadius * sphereRadius;
}

// Determines if a point is outside an AABB by checking each axis for non-containment within the AABB's min and max bounds.
bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
	/******Student:Assignment1******/

	//the physical objects largely occupy a relatively small portion of the total world space with a smaller chance of any given point being inside an object's AABB.
	//so uased a non-containment test which can quickly dismiss a large number of potential interactions, reducing computational overhead.
	return !(point.x < aabbMin.x || point.x > aabbMax.x ||
		point.y < aabbMin.y || point.y > aabbMax.y ||
		point.z < aabbMin.z || point.z > aabbMax.z);
}

// Ray-plane intersection test that calculates the intersection 't' value, handling cases where the ray is parallel to the plane.
bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
	const Vector4& plane, float& t, float epsilon)
{
	++Application::mStatistics.mRayPlaneTests;
	/******Student:Assignment1******/

	Vector3 planeNormal(plane.x, plane.y, plane.z);

	// calc the dot product of the plane's normal and the ray's direction
	float denominator = planeNormal.Dot(rayDir);

	// make sure the ray is not parallel to the plane
	if (std::abs(denominator) < epsilon) {
		t = Math::PositiveMax();
		return false;
	}

	// at the intersection
	t = (plane.w - planeNormal.Dot(rayStart)) / denominator;

	return t >= 0;
}

// Ray-triangle intersection test that first checks for intersection with the plane of the triangle, then uses barycentric coordinates for containment within the triangle.
bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
	const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
	float& t, float triExpansionEpsilon)
{
	++Application::mStatistics.mRayTriangleTests;
	/******Student:Assignment1******/

	//find intersection
	Vector3 normal = (triP1 - triP0).Cross(triP2 - triP0).Normalized();
	float d = normal.Dot(triP0);
	Vector4 plane(normal.x, normal.y, normal.z, d);
	t = Math::PositiveMax();//init

	//check plane <-> ray first.
	if (!RayPlane(rayStart, rayDir, plane, t)) {
		return false;
	}

	//containment test (barycentric coordinates)
	Vector3 p = rayStart + rayDir * t; // intersection point with the plane
	float u, v, w;
	// Check if 'p' is within the expanded triangle
	return BarycentricCoordinates(p, triP0, triP1, triP2, u, v, w, triExpansionEpsilon);
}

// Ray-sphere intersection test using quadratic formula, checks for cases where the ray starts inside or intersects the sphere.
bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
	const Vector3& sphereCenter, float sphereRadius,
	float& t)
{
	++Application::mStatistics.mRaySphereTests;

	/******Student:Assignment1******/
	Vector3 m = sphereCenter-rayStart;
	float a = rayDir.LengthSq();
	float negateB = m.Dot(rayDir);
	float c = m.LengthSq() - sphereRadius * sphereRadius;
	t = 0.f;

	// (optimization) check if the ray starts inside (or on the surface of) the sphere
	if (c <= 0.f) {
		return true;
	}

	float discriminant = negateB * negateB - a * c;

	if (discriminant < 0) {
		return false; // no real root
	}

	float sqrtDiscriminant = static_cast<float>(sqrt(discriminant));
	float t1 = (negateB - sqrtDiscriminant) / a;
	float t2 = (negateB + sqrtDiscriminant) / a;

	// intersection points behind the ray
	if (t1 < 0.f && t2 < 0.f) {
		return false;
	}

	if (t1 < 0.f) {
		t = t2; // t1 is begind the ray
	}
	else if (t2 < 0.f) {
		t = t1; // t2 is behind the ray
	}
	else {
		t = std::min(t1, t2); // if both valid, pick the closer one
	}
	return true;
}

// Ray-AABB intersection test iteratively checks each axis and updates the entry and exit points (tMin and tMax).
bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
	const Vector3& aabbMin, const Vector3& aabbMax, float& t)
{
	++Application::mStatistics.mRayAabbTests;
	/******Student:Assignment1******/
	float tMin = 0.0f; // start of intersection
	float tMax = std::numeric_limits<float>::max(); // end of intersection
	t = 0.f;

	// for each axis (x, y, and z)
	for (int i = 0; i < 3; ++i) {
		// ray is parallel to slab.
		if (std::abs(rayDir[i]) < std::numeric_limits<float>::epsilon()) {
			// then, check if the ray's origin is outside the AABB on this axis.
			if (rayStart[i] < aabbMin[i] || rayStart[i] > aabbMax[i]) {
				return false;
			}
		}
		else {
			float invRayDirComponent = 1.f / rayDir[i];
			float tNear = (aabbMin[i] - rayStart[i]) * invRayDirComponent;
			float tFar = (aabbMax[i] - rayStart[i]) * invRayDirComponent;

			// If the ray direction component along this axis is negative, the intersections with near and far sides are swapped.
			if (tNear > tFar) {
				std::swap(tNear, tFar);
			}

			tMin = std::max(tMin, tNear);
			tMax = std::min(tMax, tFar);
			// Check for valid intersection conditions
			// tMin > tMax: This condition checks if the ray exits the AABB before it enters, indicating no intersection.
			// tMax < 0: This checks if the earliest exit point (tMax) is behind the ray's origin. 
			//           If true, it means the AABB is entirely behind the ray, and thus, there is no forward intersection.
			if (tMin > tMax || tMax < 0) {
				return false; // No valid intersection
			}
		}
	}

	// At this point, an intersection is confirmed.
	// If tMin is negative, it means the ray starts inside the AABB, so we set t to 0.
	// Otherwise, set t to tMin, the first point of intersection.
	t = (tMin < 0) ? 0 : tMin; 
	return true;
}

IntersectionType::Type PlaneTriangleWithIntersections(const Vector4& plane, const Vector3& triP0, const Vector3& triP1, const Vector3& triP2, float t[3], IntersectionType::Type classifications[3], float epsilon, bool earlyExit)
{
	classifications[0] = PointPlane(triP0, plane, epsilon);
	classifications[1] = PointPlane(triP1, plane, epsilon);
	classifications[2] = PointPlane(triP2, plane, epsilon);

	// if all vertices are on one side of the plane (or all coplanar), return that classification
	if (classifications[0] == classifications[1] && classifications[1] == classifications[2]) {
		return classifications[0];
	}

	// straddling
	// check edge intersections only if one vertex is inside and the other is outside
	bool edgeIntersects = false;
	if ((classifications[0] == IntersectionType::Inside && classifications[1] == IntersectionType::Outside) ||
		(classifications[0] == IntersectionType::Outside && classifications[1] == IntersectionType::Inside)) {
		edgeIntersects |= EdgeIntersectsPlane(triP0, triP1, plane, t[0], epsilon);
	}
	if ((!earlyExit || !edgeIntersects) &&
		((classifications[1] == IntersectionType::Inside && classifications[2] == IntersectionType::Outside) ||
			(classifications[1] == IntersectionType::Outside && classifications[2] == IntersectionType::Inside))) {
		edgeIntersects |= EdgeIntersectsPlane(triP1, triP2, plane, t[1], epsilon);
	}
	if ((!earlyExit || !edgeIntersects) &&
		((classifications[2] == IntersectionType::Inside && classifications[0] == IntersectionType::Outside) ||
			(classifications[2] == IntersectionType::Outside && classifications[0] == IntersectionType::Inside))) {
		edgeIntersects |= EdgeIntersectsPlane(triP2, triP0, plane, t[2], epsilon);
	}

	if (edgeIntersects) {
		return IntersectionType::Overlaps;
	}
	// if no edge intersection, either inside or outside
	if (classifications[0] == IntersectionType::Inside ||
		classifications[1] == IntersectionType::Inside ||
		classifications[2] == IntersectionType::Inside) {
		return IntersectionType::Inside;
	}
	return IntersectionType::Outside;
}

/**
  * This function determines if a triangle intersects with a plane by evaluating the positions
  * of its vertices relative to the plane. The current implementation calculates 't1', 't2', and 't3'
  * for the exact intersection points on each edge of the triangle. This level of detail is not required
  * for the current assignment (which only needs a true/false indication of intersection) but might be
  * useful for later assignments requiring precise intersection coordinates.
  */
IntersectionType::Type PlaneTriangle(const Vector4& plane,
	const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
	float epsilon)
{
	++Application::mStatistics.mPlaneTriangleTests;

	/******Student:Assignment1******/
	float t[3]{ -1.f,-1.f,-1.f };
	IntersectionType::Type types[3]{ IntersectionType::NotImplemented ,IntersectionType::NotImplemented ,IntersectionType::NotImplemented };

	return PlaneTriangleWithIntersections(plane, triP0,triP1,triP2,t,types,epsilon);
}

// Plane-sphere intersection test by finding the closest point on the plane to the sphere's center and comparing it to the sphere's radius.
IntersectionType::Type PlaneSphere(const Vector4& plane,
	const Vector3& sphereCenter, float sphereRadius)
{
	++Application::mStatistics.mPlaneSphereTests;
	/******Student:Assignment1******/
	Vector3 planeNormal(plane.x, plane.y, plane.z);

	// closest point on the plane to the sphere's center
	Vector3 closestPoint = ProjectPointOnPlane(sphereCenter, planeNormal, plane.w);

	// if the closest point on the plane is inside the sphere
	if (PointSphere(closestPoint, sphereCenter, sphereRadius)) {
		return IntersectionType::Overlaps;
	}

	//either inside or outside
	//inside:  the sphere is on the side to which the plane normal points 
	//outside: the sphere is on the opposite side
	return (sphereCenter.Dot(planeNormal)> plane.w) ? IntersectionType::Inside : IntersectionType::Outside;
}

// Plane-AABB intersection test using the projection interval radius method to determine if the AABB overlaps, is inside, or outside the plane.
IntersectionType::Type PlaneAabb(const Vector4& plane,
	const Vector3& aabbMin, const Vector3& aabbMax)
{
	++Application::mStatistics.mPlaneAabbTests;
	/******Student:Assignment1******/
	Vector3 planeNormal(plane.x, plane.y, plane.z);

	// calc the center and half-extents of the AABB
	Vector3 center = (aabbMin + aabbMax) * 0.5f;
	Vector3 halfExtents = (aabbMax - aabbMin) * 0.5f;

	// calc the projection interval radius of b onto L(t) = b.c + t * p.n
	float radius = halfExtents.x * abs(planeNormal.x) +
		halfExtents.y * abs(planeNormal.y) +
		halfExtents.z * abs(planeNormal.z);

	// distance between box center <-> plane
	float distance = CalcSignedDistanceFromPointToPlane(center, plane);

	// intersection when distance within [-rad, +rad]
	if (abs(distance)<= radius) {
		return IntersectionType::Overlaps;
	}
	else if (distance > 0) {
		return IntersectionType::Inside;
	}
	else {
		return IntersectionType::Outside;
	}
}

// Frustum-triangle intersection test by classifying the triangle vertices against each plane of the frustum.
IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
	const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
	float epsilon)
{
	++Application::mStatistics.mFrustumTriangleTests;
	/******Student:Assignment1******/
	bool allInside = true;

	// for each plane of the frustum
	for (int i{}; i < 6; ++i) {                                   
		IntersectionType::Type classificationP0 = PointPlane(triP0, planes[i], epsilon);
		IntersectionType::Type classificationP1 = PointPlane(triP1, planes[i], epsilon);
		IntersectionType::Type classificationP2 = PointPlane(triP2, planes[i], epsilon);

		// if all vertices are outside this plane, the triangle is outside
		if (classificationP0 == IntersectionType::Outside &&
			classificationP1 == IntersectionType::Outside &&
			classificationP2 == IntersectionType::Outside) {
			return IntersectionType::Outside;
		}

		if (classificationP0 != IntersectionType::Inside ||
			classificationP1 != IntersectionType::Inside ||
			classificationP2 != IntersectionType::Inside) {
			allInside = false;
		}
	}

	// if all vertices are inside all planes
	if (allInside) {
		return IntersectionType::Inside;
	}

	// otherwise, the triangle overlaps the frustum
	return IntersectionType::Overlaps;
}

// Frustum-sphere intersection test by classifying the sphere against each plane of the frustum.
IntersectionType::Type FrustumSphere(const Vector4 planes[6],
	const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
	++Application::mStatistics.mFrustumSphereTests;
	/******Student:Assignment1******/

	bool isInside = true;

	//last known axis where sphere is outside
	size_t startAxis = lastAxis < 6 ? lastAxis : 0;
	lastAxis = 6; // reset

	for (size_t i = 0; i < 6; ++i) {
		size_t planeIndex = (startAxis + i) % 6; // start from startAxis & cycle through planes

		// classify the sphere against each plane
		IntersectionType::Type classification = PlaneSphere(planes[planeIndex], sphereCenter, sphereRadius);

		// if sphere is outside just one plane, it's outside the whole frustum
		if (classification == IntersectionType::Outside) {
			lastAxis = planeIndex; // update lastAxis
			return IntersectionType::Outside;
		}
		// sphere not inside a plane, it's not fully inside the frustum
		if (classification != IntersectionType::Inside) {
			isInside = false;
		}
	}
	// the sphere inside all planes
	if (isInside) {
		return IntersectionType::Inside;
	}

	// otherwise, the sphere overlaps the frustum
	return IntersectionType::Overlaps;
}

// Frustum-AABB intersection test by classifying the AABB against each plane of the frustum
IntersectionType::Type FrustumAabb(const Vector4 planes[6],
	const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
	++Application::mStatistics.mFrustumAabbTests;
	/******Student:Assignment1******/
	bool isInside = true;

	// assign the last axis made the AABB as outside
	size_t startAxis = lastAxis < 6 ? lastAxis : 0;
	lastAxis = 6; // reset

	for (size_t i{}; i < 6; ++i) {
		size_t planeIndex =  (startAxis + i) % 6; // cycle through planes starting from startAxis

		// classify the AABB against the current plane        (frustum planes are facing inward)
		IntersectionType::Type classification = PlaneAabb(planes[planeIndex], aabbMin, aabbMax);

		// if an AABB outside the plane, outside the frustum as well
		if (classification == IntersectionType::Outside) {
			lastAxis = planeIndex; // update lastAxis
			return IntersectionType::Outside;
		}

		// if not inside even one plane, the AABB cannot be fully inside the frustum
		if (classification != IntersectionType::Inside) {//equivalent to check IntersectionType::Overlaps
			isInside = false;
		}
	}

	//completely inside
	if (isInside) {
		return IntersectionType::Inside;
	}

	// otherwise, the AABB overlaps the frustum
	return IntersectionType::Overlaps;
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
	const Vector3& sphereCenter1, float sphereRadius1)
{
	++Application::mStatistics.mSphereSphereTests;
	/******Student:Assignment1******/

	// calc the distance between the centers of the two spheres
	float distanceSquared = (sphereCenter0 - sphereCenter1).LengthSq();

	// calc the sum of the radii, squared
	float radiiSumSquared = (sphereRadius0 + sphereRadius1) * (sphereRadius0 + sphereRadius1);

	// containment check
	return distanceSquared <= radiiSumSquared;
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
	const Vector3& aabbMin1, const Vector3& aabbMax1)
{
	++Application::mStatistics.mAabbAabbTests;
	/******Student:Assignment1******/
	// non containment check along each axis
	if (aabbMax0.x < aabbMin1.x || aabbMin0.x > aabbMax1.x) return false; // separation along X-axis
	if (aabbMax0.y < aabbMin1.y || aabbMin0.y > aabbMax1.y) return false; // separation along Y-axis
	if (aabbMax0.z < aabbMin1.z || aabbMin0.z > aabbMax1.z) return false; // separation along Z-axis

	// otherwise, no separation, meaning they intersect
	return true;
}
