/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Gjk.cpp
Purpose: Implements the Gilbert-Johnson-Keerthi (GJK) algorithm
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
		  OpenGL Info: 4.6.0 - Build 31.0.101.3413
		  Version: 4.6.0 - Build 31.0.101.3413
		  Vendor: Intel
		  Renderer: Intel(R) UHD Graphics
Project: byeonggyu.park_CS350_5
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 4/24/2024
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"
#include <array>

//-----------------------------------------------------------------------------SupportShape
/******Student:Assignment5******/
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const
{
	Vector3 centroid = Vector3::cZero;

	for (const Vector3& p : localPoints) {
		centroid += p;
	}

	centroid /= static_cast<float>(localPoints.size());

	return TransformPoint(transform, centroid);
}

Vector3 SupportShape::Support(const Vector3& worldDirection, const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform) const
{
	Vector3 result = Vector3::cZero;
	Vector3 center = GetCenter(localPoints, Matrix4::cIdentity);

	Vector3 locDir = TransformNormal(localToWorldTransform.Inverted(), worldDirection);

	float furthest{ -Math::PositiveMax() };
	for (const Vector3& p : localPoints) {
		float dist = Math::Dot(locDir, p - center);
		if (dist > furthest) {// when there is a tie, pick the first point
			furthest = dist;
			result = p;
		}
	}

	return Math::TransformPoint(localToWorldTransform, result);
}

/******Student:Assignment5******/
void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
	for (const Vector3& p : localPoints) {
		gDebugDrawer->DrawPoint(p).SetTransform(localToWorldTransform).Color(color);
	}
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
	return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
	return SupportShape::Support(worldDirection, mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
	SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
	mScale = Vector3(1);
	mRotation = Matrix3::cIdentity;
	mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const
{
	return mSphere.mCenter;
}

/******Student:Assignment5******/
Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
	Vector3 normalizedDirection = worldDirection;
	float lengthSq = normalizedDirection.AttemptNormalize();

	// lengthSq zero or too small
	if (lengthSq < Math::DebugEpsilon() * Math::DebugEpsilon()) {
		// by default, returns the center point
		return GetCenter();
	}

	return GetCenter() + mSphere.mRadius * normalizedDirection;
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
	DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
	shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const
{
	return mTranslation;
}

/******Student:Assignment5******/
Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
	//(1) world -> local
	Vector3 localDir = Math::Transform(mRotation.Inverted(), worldDirection);

	// sign for each component of the local support vector
	Vector3 sign{ 
		Math::GetSign(localDir.x),	
		Math::GetSign(localDir.y),	
		Math::GetSign(localDir.z) 
	};

	// basis vectors for each axis, (adjusted by half-extent)
	Vector3 basisX = mRotation.Basis(0) * (mScale.x * 0.5f);
	Vector3 basisY = mRotation.Basis(1) * (mScale.y * 0.5f);
	Vector3 basisZ = mRotation.Basis(2) * (mScale.z * 0.5f);

	// by default, at the OBB's center
	Vector3 supportPoint = GetCenter();

	//(2) transform the support point from local -> world coordinates using 'basis vector's computed above
	//	  this ensures the support point is in the same direction as the support vector
	supportPoint += sign.x * basisX;
	supportPoint += sign.y * basisY;
	supportPoint += sign.z * basisZ;

	return supportPoint;
}


void ObbSupportShape::DebugDraw(const Vector4& color) const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
	shape.Color(color);
	shape.SetTransform(transform);
}

// utility function to compute a point from barycentric coordinates for an edge
Vector3 Gjk::ComputeEdgePoint(const EdgeBarycentric& edge, const Vector3& p0, const Vector3& p1) {
	return p0 + edge.v * (p1 - p0);
}

// utility function to compute a point from barycentric coordinates for a triangle
Vector3 Gjk::ComputeTrianglePoint(const TriangleBarycentric& tri, const Vector3& p0, const Vector3& p1, const Vector3& p2) {
	return tri.u * p0 + tri.v * p1 + tri.w * p2; // point within the triangle
}

Gjk::EdgeBarycentric Gjk::CalculateEdgeBarycentric(const Vector3& q, const Vector3& p0, const Vector3& p1) {
	float u{}, v{};
	BarycentricCoordinates(q, p0, p1, u, v);
	return EdgeBarycentric(u, v);
}

Gjk::TriangleBarycentric Gjk::CalculateTriangleBarycentric(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2) {
	float u{}, v{}, w{};
	BarycentricCoordinates(q, p0, p1, p2, u, v, w);
	return TriangleBarycentric(u, v, w);
}

bool Gjk::IsQueryPointOutsideTriangleVR(const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& reference) {
	Vector3 triangleNormal = Math::Cross(b - a, c - a);
	// flip, if the normal points inward
	if (Math::Dot(triangleNormal, reference - a) > 0) {
		triangleNormal = -triangleNormal;
	}
	return Math::Dot(triangleNormal, q - a) > 0;
}

//------------------------------------------------------------ Voronoi Region Tests
/******Student:Assignment5******/
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
	size_t& newSize, int newIndices[4],
	Vector3& closestPoint, Vector3& searchDirection)
{
	// just one vertex
	newSize = 1;
	newIndices[0] = 0;

	closestPoint = p0;
	searchDirection = { q - p0 };

	return VoronoiRegion::Point0;
}

/******Student:Assignment5******/
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
	size_t& newSize, int newIndices[4],
	Vector3& closestPoint, Vector3& searchDirection)
{
	float u{}, v{};
	BarycentricCoordinates(q, p0, p1, u, v);

	//i. VR(p0)
	if (v <= 0.f) {
		// Closest to p0
		newSize = 1;
		newIndices[0] = 0;

		closestPoint = p0;
		searchDirection = q - p0;

		return VoronoiRegion::Point0;
	}
	//ii. VR(p1)
	else if (u <= 0.f)
	{
		newSize = 1;
		newIndices[0] = 1;

		closestPoint = p1;
		searchDirection = q - p1;

		return VoronoiRegion::Point1;
	}

	//iii. VR(p0p1)
	closestPoint = p0 * u + p1 * v;

	newSize = 2;
	newIndices[0] = 0;
	newIndices[1] = 1;

	searchDirection = q - closestPoint;

	return VoronoiRegion::Edge01;
}

/******Student:Assignment5******/
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
	size_t& newSize, int newIndices[4],
	Vector3& closestPoint, Vector3& searchDirection)
{
	EdgeBarycentric ab = CalculateEdgeBarycentric(q, p0, p1);
	EdgeBarycentric bc = CalculateEdgeBarycentric(q, p1, p2);
	EdgeBarycentric ca = CalculateEdgeBarycentric(q, p2, p0);

	//1. VR(p0)
	if (ab.v <= 0 && ca.u <= 0) {
		closestPoint = p0;
		newSize = 1;
		newIndices[0] = 0;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point0;
	}
	//2. VR(p1)
	if (bc.v <= 0 && ab.u <= 0) {
		closestPoint = p1;
		newSize = 1;
		newIndices[0] = 1;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point1;
	}
	//3. VR(p2)
	if (ca.v <= 0 && bc.u <= 0) {
		closestPoint = p2;
		newSize = 1;
		newIndices[0] = 2;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point2;
	}

	// obtain triangle barycentric coords for edge VRs
	TriangleBarycentric triCoords = CalculateTriangleBarycentric(q, p0, p1, p2);

	//4. VR(p0p1)
	if (triCoords.w <= 0 && ab.u > 0 && ab.v > 0) {
		closestPoint = ComputeEdgePoint(ab, p0, p1);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 0;
		newIndices[1] = 1;
		return VoronoiRegion::Edge01;
	}
	//5. VR(p1p2)
	if (triCoords.u <= 0 && bc.u > 0 && bc.v > 0) {
		closestPoint = ComputeEdgePoint(bc, p1, p2);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 1;
		newIndices[1] = 2;
		return VoronoiRegion::Edge12;
	}
	//6. VR(p2p0)
	if (triCoords.v <= 0 && ca.u > 0 && ca.v > 0) {
		closestPoint = ComputeEdgePoint(ca, p2, p0);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 0;
		newIndices[1] = 2;
		return VoronoiRegion::Edge02;
	}
	//7. VR(p0p1p2)
	closestPoint = triCoords.u * p0 + triCoords.v * p1 + triCoords.w * p2;
	searchDirection = q - closestPoint;
	newSize = 3;
	newIndices[0] = 0;
	newIndices[1] = 1;
	newIndices[2] = 2;
	return VoronoiRegion::Triangle012;
}

/******Student:Assignment5******/
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d,
	size_t& newSize, int newIndices[4],
	Vector3& closestPoint, Vector3& searchDirection)
{
	EdgeBarycentric ab = CalculateEdgeBarycentric(q, a, b);
	EdgeBarycentric ac = CalculateEdgeBarycentric(q, a, c);
	EdgeBarycentric ad = CalculateEdgeBarycentric(q, a, d);
	EdgeBarycentric bc = CalculateEdgeBarycentric(q, b, c);
	EdgeBarycentric bd = CalculateEdgeBarycentric(q, b, d);
	EdgeBarycentric cd = CalculateEdgeBarycentric(q, c, d);

	//1. VR(p0)
	if (ab.v <= 0 && ad.v <= 0 && ac.v <= 0) {
		closestPoint = a;
		newSize = 1;
		newIndices[0] = 0;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point0;
	}
	//2. VR(p1)
	if (ab.u <= 0 && bc.v <= 0 && bd.v <= 0) {
		closestPoint = b;
		newSize = 1;
		newIndices[0] = 1;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point1;
	}
	//3. VR(p2)
	if (ac.u <= 0 && bc.u <= 0 && cd.v <= 0) {
		closestPoint = c;
		newSize = 1;
		newIndices[0] = 2;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point2;
	}
	//4. VR(p3)
	if (ad.u <= 0 && bd.u <= 0 && cd.u <= 0) {
		closestPoint = d;
		newSize = 1;
		newIndices[0] = 3;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point3;
	}

	// triangle barycentric coords for edge VRs
	TriangleBarycentric abd = CalculateTriangleBarycentric(q, a, b, d);
	TriangleBarycentric adc = CalculateTriangleBarycentric(q, a, d, c);
	TriangleBarycentric bcd = CalculateTriangleBarycentric(q, b, c, d);
	TriangleBarycentric acb = CalculateTriangleBarycentric(q, a, c, b);

	//5. VR(p0p1)
	if (abd.w < 0 && acb.v < 0 && ab.u > 0 && ab.v > 0) {
		closestPoint = ComputeEdgePoint(ab, a, b);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 0;
		newIndices[1] = 1;
		return VoronoiRegion::Edge01;
	}
	//6. VR(p0p2)
	if (acb.w < 0 && adc.v < 0 && ac.u > 0 && ac.v > 0) {
		closestPoint = ComputeEdgePoint(ac, a, c);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 0;
		newIndices[1] = 2;
		return VoronoiRegion::Edge02;
	}
	//7. VR(p0p3)
	if (abd.v < 0 && adc.w < 0 && ad.u > 0 && ad.v > 0) {
		closestPoint = ComputeEdgePoint(ad, a, d);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 0;
		newIndices[1] = 3;
		return VoronoiRegion::Edge03;
	}
	//8. VR(p1p2)
	if (acb.u < 0 && bcd.w < 0 && bc.u > 0 && bc.v > 0) {
		closestPoint = ComputeEdgePoint(bc, b, c);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 1;
		newIndices[1] = 2;
		return VoronoiRegion::Edge12;
	}
	//9. VR(p1p3)
	if (abd.u < 0 && bcd.v < 0 && bd.u > 0 && bd.v > 0) {
		closestPoint = ComputeEdgePoint(bd, b, d);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 1;
		newIndices[1] = 3;
		return VoronoiRegion::Edge13;
	}
	//10. VR(p2p3)
	if (bcd.u < 0 && adc.u < 0 && cd.u > 0 && cd.v > 0) {
		closestPoint = ComputeEdgePoint(cd, c, d);
		searchDirection = q - closestPoint;
		newSize = 2;
		newIndices[0] = 2;
		newIndices[1] = 3;
		return VoronoiRegion::Edge23;
	}

	//11. VR(p0p1p3) : ABD
	if (abd.u > 0 && abd.v > 0 && abd.w > 0 && IsQueryPointOutsideTriangleVR(q, a, b, d, c) == true) {
		closestPoint = ComputeTrianglePoint(abd, a, b, d);
		searchDirection = q - closestPoint;
		newSize = 3;
		newIndices[0] = 0;
		newIndices[1] = 1;
		newIndices[2] = 3;
		return VoronoiRegion::Triangle013;
	}
	//12. VR(p0p2p3) :ADC
	if (adc.u > 0 && adc.v > 0 && adc.w > 0 && IsQueryPointOutsideTriangleVR(q, a, d, c, b) == true) {
		closestPoint = ComputeTrianglePoint(adc, a, d, c);
		searchDirection = q - closestPoint;
		newSize = 3;
		newIndices[0] = 0;
		newIndices[1] = 2;
		newIndices[2] = 3;
		return VoronoiRegion::Triangle023;
	}
	//13. VR(p1p2p3) :BCD
	if (bcd.u > 0 && bcd.v > 0 && bcd.w > 0 && IsQueryPointOutsideTriangleVR(q, b, c, d, a) == true) {
		closestPoint = ComputeTrianglePoint(bcd, b, c, d);
		searchDirection = q - closestPoint;
		newSize = 3;
		newIndices[0] = 1;
		newIndices[1] = 2;
		newIndices[2] = 3;
		return VoronoiRegion::Triangle123;
	}
	//14. VR(p0p2p1) :ACB
	if (acb.u > 0 && acb.v > 0 && acb.w > 0 && IsQueryPointOutsideTriangleVR(q, a, c, b, d) == true) {
		closestPoint = ComputeTrianglePoint(acb, a, c, b);
		searchDirection = q - closestPoint;
		newSize = 3;
		newIndices[0] = 0;
		newIndices[1] = 1;
		newIndices[2] = 2;
		return VoronoiRegion::Triangle012;
	}
	//15. VR(ABCD) : within the tetrahedron
	searchDirection = Vector3::cZero;
	newSize = 4;
	newIndices[0] = 0;
	newIndices[1] = 1;
	newIndices[2] = 2;
	newIndices[3] = 3;
	closestPoint = q;
	return VoronoiRegion::Tetrahedra0123;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
	Vector3 d{shapeA->GetCenter() - shapeB->GetCenter()};

	if (d.Length() <= std::numeric_limits<float>::epsilon()) {
		d = Vector3(-1, 0, 0);
	}

	std::array<CsoPoint, 4> currentSimplex;
	closestPoint = currentSimplex[0]=ComputeSupport(shapeA, shapeB, d);//initial support point
	
	//p --> q
	d = Vector3::cZero - closestPoint.mCsoPoint;

	size_t currSimplexSize{ 1 };//1 = 0-simplex
	size_t newSimplexSize{};

	unsigned itr{};

	while (itr < maxIterations) {
		//find VR and evolve current simplex
		if (EvolveSimplexAndCheckIntersection(currentSimplex, currSimplexSize, newSimplexSize, closestPoint, d) == true) {
			return true;//Minkowski Diff contains the origin (=shapes intersect)
		}

		//add the new support point to the current simplex
		CsoPoint newSupportPoint = ComputeSupport(shapeA, shapeB, d);
		currSimplexSize = newSimplexSize;
		currentSimplex[currSimplexSize++] = newSupportPoint;

		//Exit Condition (2), (s-p).(q-p) <=0
		if (Math::Dot(newSupportPoint.mCsoPoint - closestPoint.mCsoPoint, d) <= epsilon) {
			break; // converge or no progress
		}
		++itr;
	}
	DetermineClosestCsoPoint(newSimplexSize, closestPoint, currentSimplex);

	return false; // No intersection
}


/******Student:Assignment5******/
Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
	CsoPoint supportPoint;

	supportPoint.mPointA = shapeA->Support(direction);
	supportPoint.mPointB = shapeB->Support(-direction);

	// Minkowski difference
	supportPoint.mCsoPoint = supportPoint.mPointA - supportPoint.mPointB;

	return supportPoint;
}

void Gjk::DetermineClosestCsoPoint(size_t newSimplexSize, CsoPoint& closestPoint, const std::array<CsoPoint, 4>& currentSimplex) const{
	switch (newSimplexSize)
	{
	case 1:
		closestPoint = currentSimplex[0];
		break;

	case 2:
	{
		float u{}, v{};
		BarycentricCoordinates(closestPoint.mCsoPoint, currentSimplex[0].mCsoPoint, currentSimplex[1].mCsoPoint, u, v);

		closestPoint.mCsoPoint = closestPoint.mCsoPoint;
		closestPoint.mPointA = currentSimplex[0].mPointA + v * (currentSimplex[1].mPointA - currentSimplex[0].mPointA);
		closestPoint.mPointB = currentSimplex[0].mPointB + v * (currentSimplex[1].mPointB - currentSimplex[0].mPointB);
	}
	break;

	case 3:
	{
		float u{}, v{}, w{};
		BarycentricCoordinates(closestPoint.mCsoPoint, currentSimplex[0].mCsoPoint, currentSimplex[1].mCsoPoint, currentSimplex[2].mCsoPoint, u, v, w);

		closestPoint.mCsoPoint = closestPoint.mCsoPoint;
		closestPoint.mPointA = currentSimplex[0].mPointA * u + currentSimplex[1].mPointA * v + currentSimplex[2].mPointA * w;
		closestPoint.mPointB = currentSimplex[0].mPointB * u + currentSimplex[1].mPointB * v + currentSimplex[2].mPointB * w;
	}
	break;

	default:
		break;
	}

}

bool Gjk::EvolveSimplexAndCheckIntersection(std::array<CsoPoint, 4>& currentSimplex, size_t currSimplexSize, size_t& newSimplexSize, CsoPoint& closestPoint, Vector3& d) const{
	int newIndices[4]{ 0, };

	switch (currSimplexSize) {
	case 1:
		IdentifyVoronoiRegion(Vector3::cZero, currentSimplex[0].mCsoPoint, newSimplexSize, newIndices, closestPoint.mCsoPoint, d);
		break;
	case 2:
		IdentifyVoronoiRegion(Vector3::cZero, currentSimplex[0].mCsoPoint, currentSimplex[1].mCsoPoint, newSimplexSize, newIndices, closestPoint.mCsoPoint, d);
		break;
	case 3:
		IdentifyVoronoiRegion(Vector3::cZero, currentSimplex[0].mCsoPoint, currentSimplex[1].mCsoPoint, currentSimplex[2].mCsoPoint, newSimplexSize, newIndices, closestPoint.mCsoPoint, d);
		break;
	case 4:
		IdentifyVoronoiRegion(Vector3::cZero, currentSimplex[0].mCsoPoint, currentSimplex[1].mCsoPoint, currentSimplex[2].mCsoPoint, currentSimplex[3].mCsoPoint, newSimplexSize, newIndices, closestPoint.mCsoPoint, d);
		break;
	default:
		break;
	}
	d.AttemptNormalize();

	//Exit Condition (1), Q-P = 0
	if (Vector3::cZero == closestPoint.mCsoPoint) {
		return true; // intersect!
	}

	for (size_t i{}; i < newSimplexSize; ++i) {
		currentSimplex[i] = currentSimplex[newIndices[i]];
	}

	return false;
}
