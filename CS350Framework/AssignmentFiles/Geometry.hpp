/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Geometry.hpp
Purpose: Defines various geometric intersection and containment tests, including functions for calculating distances, checking collisions, and classifying geometric primitives (like points, rays, planes, spheres, AABBs) relative to each other.
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
          OpenGL Info: 4.6.0 - Build 31.0.101.3413
          Version: 4.6.0 - Build 31.0.101.3413
          Vendor: Intel
          Renderer: Intel(R) UHD Graphics
Project: byeonggyu.park_CS350_2
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 1/26/2024
End Header -------------------------------------------------------*/

#pragma once

#include "Math/Math.hpp"

// Using directives
using Math::Vector3;
using Math::Vector4;

namespace IntersectionType
{
  enum Type {Coplanar = 0, Outside, Inside, Overlaps, NotImplemented};
  static const char* Names[] = {"Coplanar", "Outside", "Inside", "Overlaps", "NotImplemented"};
}

// Helper function to compute the signed distance between a point and a plane.
// Parameters:
//   point - A 3D point represented by its x, y, and z coordinates.
//   plane - A 4D vector representing the plane. The first three components (x, y, z) 
//           are the normal vector of the plane, and the fourth component (w) is the 
//           signed distance from the origin to the plane along its normal vector.
// Returns:
//   The signed distance from the point to the plane.
float CalcSignedDistanceFromPointToPlane(const Vector3& point, const Vector4& plane);

// Helper function to checks if a line segment (edge) intersects with a plane.
// Parameters:
//   edgeStart - Start point of the edge in 3D space.
//   edgeEnd - End point of the edge in 3D space.
//   plane - Plane represented by a 4D vector (normal and distance from origin).
//   t - Intersection parameter along the edge where the intersection occurs.
//   epsilon - Small threshold for numerical precision.
// Returns:
//   True if the edge intersects the plane within the segment; false otherwise.
bool EdgeIntersectsPlane(const Vector3& edgeStart, const Vector3& edgeEnd, const Vector4& plane, float& t, float epsilon);

// Helper function that you should likely use in several plane intersection functions. Also required for unit-tests.
Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance);

// Compute the barycentric coordinates of a point to a line. Returns false if the line is degenerate or the passed in point is outside the line.
// The given epsilon should be used to expand the line before checking if the point is outside (use it to expand the range of the barycentric coordinate checks).
// Note: The barycentric coordinates should still be filled out when the point is outside the line.
bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
                            float& u, float& v, float epsilon = 0.0f);

// Compute the barycentric coordinates of a point to a triangle. Returns false if the triangle is degenerate or the passed in point is outside the triangle.
// The given epsilon should be used to expand the triangle before checking if the point is outside the triangle (use it to expand the range of the barycentric coordinate checks).
// Note: The barycentric coordinates should still be filled out when the point is outside the triangle.
bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
                            float& u, float& v, float& w, float epsilon = 0.0f);

//--------------------------------------------------------------------------------------------------------------------
// Point Tests
//--------------------------------------------------------------------------------------------------------------------

// Classify a point with respect to a plane. This should return Inside, Outside, or Coplanar (in the event the the point is within epsilon distance from the plane).
IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon);

// Determine if the given point is within the sphere. Remember, the surface of the sphere is considered part of the sphere.
bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius);

// Determine if the given point is within the aabb. Remember, the surface of the aabb is considered part of the aabb.
bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax);

//--------------------------------------------------------------------------------------------------------------------
// Ray Tests
//--------------------------------------------------------------------------------------------------------------------

// Ray vs. plane where the plane is represented by the general plane
// equation ax + by + cz + d = 0 (so the plane parameters are [a, b, c, d]).
// Epsilon is use to check if the ray is close enough to parallel to the plane; in this case return false.
bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
              const Vector4& plane, float& t, float epsilon = 0.0001f);

// Check if a ray hits a triangle. This should return false if the ray doesn't hit the triangle.
// If the ray does hit the triangle the t-value of where the ray hits the triangle should be filled out.
// Note: Use the triExpansionEpsilon to expand the triangle before checking for containment (in the barycentric coordinates test).
// This becomes useful and necessary later to deal with edge cases in a mesh (e.g. in the bsp tree).
bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                 float& t, float triExpansionEpsilon);

// Check if a ray hits a sphere. If the ray does hit the sphere then t should be filled out with the first time of impact.
// If the ray starts inside the sphere then t should be 0. Note: t should never be set to a negative value if this function returns true!
bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
               const Vector3& sphereCenter, float sphereRadius,
               float& t);

// Check if a ray hits an aabb. If the ray does hit the aabb then t should be filled out with the first time of impact.
// If the ray starts inside the aabb then t should be 0. Note: t should never be set to a negative value if this function returns true!
// Do not call RayPlane here (it's inefficient)!
bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
             const Vector3& aabbMin, const Vector3& aabbMax, float& t);

//--------------------------------------------------------------------------------------------------------------------
// Plane Tests
//--------------------------------------------------------------------------------------------------------------------
// not only classify a triangle against a plane but also returns intersections.
// if early exit is true, skip other intersection tests once an edge intersects.  
IntersectionType::Type PlaneTriangleWithIntersections(const Vector4& plane, const Vector3& triP0, const Vector3& triP1, const Vector3& triP2, float t[3], IntersectionType::Type classifications[3], float epsilon, bool earlyExit=true);


// Classify a triangle against a plane. This should return Coplanar only if all points are within epsilon distance from the plane.
IntersectionType::Type PlaneTriangle(const Vector4& plane,
    const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,float epsilon);

// Classify a sphere against a plane. This should return Overlaps if the sphere hits the plane. No check for Coplanar is needed.
IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter, float sphereRadius);

// Classify an aabb against a plane. This should return Overlaps if the aabb hits the plane. Do not check all 8 points of the aabb against the plane!
// You must use the 2 point method described in class (the furthest positive and negative point method). No check for Coplanar is needed.
IntersectionType::Type PlaneAabb(const Vector4& plane,
                                 const Vector3& aabbMin, const Vector3& aabbMax);

//--------------------------------------------------------------------------------------------------------------------
// Frustum Tests. Note: You cannot determine that a shape Overlaps if one plane of the frustum classifies as Overlaps.
// The shape could still be outside another plane!! Also, Coplanar checks are not necessary.
// Finally, the lastAxis variable is not needed for the first assignment, but will be needed in assignment 3.
// The last axis should be filled out with an axis that causes the test to return Outside (if there is one).
// This should also be used as the first axis you test so that temporal coherence can be used to reject after only 1 plane test the next frame.
//--------------------------------------------------------------------------------------------------------------------

// Classify a triangle against a frustum. The given epsilon should be used when classifying the triangle points against the plane.
IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                       float epsilon);


IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis);

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis);

//--------------------------------------------------------------------------------------------------------------------
// Simple primitive tests
//--------------------------------------------------------------------------------------------------------------------
bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
                  const Vector3& sphereCenter1, float sphereRadius1);

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
              const Vector3& aabbMin1, const Vector3& aabbMax1);
