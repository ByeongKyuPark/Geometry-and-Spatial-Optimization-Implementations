/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: DebugDraw.cpp
Purpose: Implementation for rendering debug visualizations of geometric shapes.
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
		  OpenGL Info: 4.6.0 - Build 31.0.101.3413
		  Version: 4.6.0 - Build 31.0.101.3413
		  Vendor: Intel
		  Renderer: Intel(R) UHD Graphics
Project: byeonggyu.park_CS350_1
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 1/27/2024
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
	mColor = Vector4(.6f);
	mMask = (unsigned int)-1;
	mTimer = 0;
	mOnTop = false;
	mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
	mColor = color;
	return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
	mOnTop = state;
	return *this;
}

DebugShape& DebugShape::Time(float time)
{
	mTimer = time;
	return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
	mMask = 1 << bitIndex;
	return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
	mTransform = transform;
	return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
	mActiveMask = (unsigned int)-1;
	mApplication = NULL;
}

void DebugDrawer::Update(float dt)
{
	std::vector<DebugShape> newShapes;
	for (size_t i = 0; i < mShapes.size(); ++i)
	{
		DebugShape& shape = mShapes[i];
		shape.mTimer -= dt;

		// If the shape still has time left then add it to the list of shapes to keep drawing,
		// anything that has a timer that ran out will not be in the new list
		if (shape.mTimer >= 0)
			newShapes.push_back(shape);
	}

	mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
	for (size_t i = 0; i < mShapes.size(); ++i)
	{
		DebugShape& shape = mShapes[i];

		// If the shape doesn't have one of the active mask bits set then don't draw it
		if ((shape.mMask & mActiveMask) == 0)
			continue;

		// If this shape always draws on top then disable depth testing
		if (shape.mOnTop)
			glDisable(GL_DEPTH_TEST);


		// Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
		float radians;
		Vector3 scale, translation, axis;
		Matrix3 rotationMat;
		shape.mTransform.Decompose(&scale, &rotationMat, &translation);
		Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
		glPushMatrix();
		// Set the transform
		glTranslatef(translation.x, translation.y, translation.z);
		glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
		glScalef(scale.x, scale.y, scale.z);

		glBegin(GL_LINES);
		glColor3fv(shape.mColor.array);

		// Draw all of the line segments of this shape
		for (size_t j = 0; j < shape.mSegments.size(); ++j)
		{
			LineSegment& segment = shape.mSegments[j];

			glVertex3fv(segment.mStart.array);
			glVertex3fv(segment.mEnd.array);
		}

		glEnd();
		glPopMatrix();

		// Make sure to re-enable depth testing
		if (shape.mOnTop)
			glEnable(GL_DEPTH_TEST);
	}
}

DebugShape& DebugDrawer::GetNewShape()
{
	mShapes.push_back(DebugShape());
	return mShapes.back();
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
	return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
	/******Student:Assignment2******/
	// Draw a simple line
	DebugShape& shape = GetNewShape();
	shape.mSegments.push_back(line);
	
	return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
	/******Student:Assignment2******/
	// Draw a ray to a given t-length. The ray must have an arrow head for visualization
	return DrawRay(GetNewShape(), ray, t);
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
	/******Student:Assignment2******/
	// Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
	// Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
	DebugShape& shape = GetNewShape();

	//orthonormal basis vectors for the disc (will be reused)

	Vector3 v{}, w{};
	// X-axis disc
	DrawDisc(shape, sphere.mCenter, Vector3::cXAxis, v,w,sphere.mRadius);
	// Y-axis disc
	DrawDisc(shape, sphere.mCenter, Vector3::cYAxis, v,w, sphere.mRadius);
	// Z-axis disc
	DrawDisc(shape, sphere.mCenter, Vector3::cZAxis, v,w, sphere.mRadius);

	if (!mApplication) 
	{
		return shape;
	}

	// draw the horizon disc
	Vector3 e = sphere.mCenter - mApplication->mCamera.mTranslation;
	float eLength = e.Length();
	
	// this horizon disc computation relies on the assumption that the camera is "external" to the sphere
	if (eLength < sphere.mRadius) {
		return shape; 
	}

	float radiusSqrd = sphere.mRadius * sphere.mRadius;
	float l = sqrtf(e.LengthSq() - radiusSqrd);
	float correctedRadius = sphere.mRadius * l / eLength;
	float z = sqrtf(radiusSqrd - correctedRadius * correctedRadius);

	e /= eLength;
	Vector3 correctedCenter = sphere.mCenter-z*e;

	// orthogonal basis for the disc
	return DrawDisc(shape, correctedCenter,e, v,w, correctedRadius);
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
	/******Student:Assignment2******/
	// Draw all edges of an aabb. Make sure to not mis-match edges!
	DebugShape& shape = GetNewShape();

	Vector3 min = aabb.GetMin();
	Vector3 max = aabb.GetMax();

	// corners
	Vector3 v0(min.x, min.y, min.z);
	Vector3 v1(max.x, min.y, min.z);
	Vector3 v2(max.x, max.y, min.z);
	Vector3 v3(min.x, max.y, min.z);
	Vector3 v4(min.x, min.y, max.z);
	Vector3 v5(max.x, min.y, max.z);
	Vector3 v6(max.x, max.y, max.z);
	Vector3 v7(min.x, max.y, max.z);

	//front
	DrawQuad(shape,v0, v1, v2, v3);
	//rear
	DrawQuad(shape, v4, v5, v6, v7);

	// edges between front & rear
	shape.mSegments.push_back(LineSegment(v0, v4));//bottom left
	shape.mSegments.push_back(LineSegment(v1, v5));//bottom right
	shape.mSegments.push_back(LineSegment(v2, v6));//top right
	shape.mSegments.push_back(LineSegment(v3, v7));//top left

	return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
	/******Student:Assignment2******/
	// Draw the 3 edges of a triangles
	DebugShape& shape = GetNewShape();
	shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[1]));
	shape.mSegments.push_back(LineSegment(triangle.mPoints[1], triangle.mPoints[2]));
	shape.mSegments.push_back(LineSegment(triangle.mPoints[2], triangle.mPoints[0]));
	return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
	/******Student:Assignment2******/
	// Draw a quad with a normal at the plane's sphereCenter.
	DebugShape& shape = GetNewShape();

	// need orthogonal basis
	Vector3 v, w;
	Vector3 normal = plane.GetNormal();
	Math::GenerateOrthonormalBasis(normal, &v, &w);
	Vector3 center = normal * plane.mData.w; 

	// corners of the quad
	Vector3 corners[4];
	corners[0] = center - v * sizeX + w * sizeY;//top-left
	corners[1] = center - v * sizeX - w * sizeY;//bottom-left
	corners[2] = center + v * sizeX - w * sizeY;//bottom-right
	corners[3] = center + v * sizeX + w * sizeY;//top-right
	DrawQuad(shape,corners[0], corners[1], corners[2], corners[3]);

	// draw the normal (length is 5, by default)
	static float constexpr PLANE_NORMAL_LENGTH = 5.f;
	return DrawRay(shape,Ray(center, normal), PLANE_NORMAL_LENGTH);
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
	/******Student:Assignment2******/
	// Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
	return DrawQuad(GetNewShape(), p0, p1, p2, p3);
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
	/******Student:Assignment2******/
	// Draw the 6 faces of the frustum using the 8 frustum points.
	// See Frustum.Set for the point order. For example, Points[4] is left-bottom-far.
	DebugShape& shape = GetNewShape();

	// near
	DrawQuad(shape, frustum.mPoints[0], frustum.mPoints[1], frustum.mPoints[2], frustum.mPoints[3]);
	// far
	DrawQuad(shape, frustum.mPoints[4], frustum.mPoints[5], frustum.mPoints[6], frustum.mPoints[7]);
	// lines between near <-> far
	shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[4]));//bottom left
	shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[5]));//bottom right
	shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[6]));//top right
	shape.mSegments.push_back(LineSegment(frustum.mPoints[3], frustum.mPoints[7]));//top left

	return shape;
}

/**
 * @brief Adds a ray representation to an existing DebugShape object.
 * This function draws a ray, including an arrowhead for visualization, and adds it to a provided DebugShape object.
 * It uses DrawDisc to draw the circular base of the arrowhead. The number of lines in the arrowhead and the segments of the disc
 * can be specified for detailed visualization.
 *
 * @param shape Reference to DebugShape object to which the ray representation will be added.
 * @param ray The ray to be drawn, defined by a start point and direction.
 * @param t Length of the ray to be drawn.
 * @param arrowHeadHeight Height of the arrowhead, defaults to 0.4
 * @param arrowLines Number of lines connecting the tip of the arrow to the disc (arrowhead base), defaults to 4.
 * @return Reference to the modified DebugShape object.
 */
DebugShape& DebugDrawer::DrawRay(DebugShape& shape, const Ray& ray, float t, float arrowHeadHeight, int arrowLines)
{
	static constexpr float ARROW_HEAD_HALF_ANGLE = 30.f;
	static constexpr int DISC_SEGMENTS = 20;//Number of segments of the disc (arrowhead base).

	Vector3 end = ray.mStart + ray.mDirection * t;
	shape.mSegments.push_back(LineSegment(ray.mStart, end));

	// Parameters for the arrowhead
	const float circleRadius = arrowHeadHeight * tanf(Math::DegToRad(ARROW_HEAD_HALF_ANGLE));
	Vector3 arrowDir = ray.mDirection.Normalized();
	Vector3 circleCenter = end - arrowDir * arrowHeadHeight;

	// Draw the circle at the base of the arrowhead using DrawDisc
	Vector3 v{}, w{};
	DrawDisc(shape, circleCenter,arrowDir, v,w, circleRadius);

	// Add the arrowhead lines
	Vector3 arrowBase = end - arrowDir * arrowHeadHeight;
	const int segmentStep = DISC_SEGMENTS / arrowLines;
	const float angleStep = Math::cTwoPi / DISC_SEGMENTS;
	for (int i{}; i < DISC_SEGMENTS; i += segmentStep)
	{
		float angle = angleStep*i;
		Vector3 pointOnCircle = arrowBase + circleRadius*(v * cosf(angle) + w * sinf(angle));
		shape.mSegments.push_back(LineSegment(end, pointOnCircle));
	}

	return shape;
}

/**
 * @brief Adds a disc to an existing DebugShape object.
 *
 * This function draws a disc and adds it to a provided DebugShape object.
 * The disc is defined by its center, normal vector, radius, and the number of segments.
 * The function also generates two orthonormal basis vectors (`v` and `w`) based on the normal vector `n`.
 *
 * @param shape The DebugShape to which the disc will be added.
 * @param center The center of the disc.
 * @param n The normal vector of the disc's plane.
 * @param v An orthonormal vector of the direction vector of the ray.
 * @param w Another orthonormal vector of the direction vector of the ray.
 * @param radius The radius of the disc.
 * @param segments The number of segments to divide the disc into. Defaults to 20.
 * @return Reference to the modified DebugShape object with the added disc.
 */
DebugShape& DebugDrawer::DrawDisc(DebugShape& shape, const Vector3& center, const Vector3&n, Vector3& v,Vector3&w, float radius, int segments)
{
	const float increment = static_cast<float>(2.f * M_PI / segments);

	Math::GenerateOrthonormalBasis(n, &v, &w);

	// Draw the disc
	for (int i{}; i < segments; ++i)
	{
		float angle = i * increment;
		float nextAngle = (i + 1) * increment;
		Vector3 start = center + radius * (v * cosf(angle)     + w * sinf(angle));
		Vector3 end   = center + radius * (v * cosf(nextAngle) + w * sinf(nextAngle));
		shape.mSegments.push_back(LineSegment(start, end));
	}

	return shape;
}

/**
 * @brief Adds a quad representation to an existing DebugShape object.
 *
 * This function draws a quadrilateral (quad) by adding its edges as line segments to the provided DebugShape object.
 * It enhances the existing DebugShape by including the visual representation of a quad without the need to create a new shape.
 * The quad is defined by four vertices, and the edges are drawn between these vertices in a consecutive manner.
 *
 * @param shape Reference to DebugShape object to which the quad will be added.
 * @param p0 The first vertex of the quad.
 * @param p1 The second vertex of the quad, connected to the first.
 * @param p2 The third vertex of the quad, connected to the second.
 * @param p3 The fourth vertex of the quad, connected to the third and back to the first.
 * @return Reference to the modified DebugShape object with the added quad.
 */
DebugShape& DebugDrawer::DrawQuad(DebugShape& shape,const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
	shape.mSegments.push_back(LineSegment(p0, p1));
	shape.mSegments.push_back(LineSegment(p1, p2));
	shape.mSegments.push_back(LineSegment(p2, p3));
	shape.mSegments.push_back(LineSegment(p3, p0));
	return shape;
}