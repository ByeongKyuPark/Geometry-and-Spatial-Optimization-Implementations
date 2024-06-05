/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: DebugDraw.hpp
Purpose: Provides the functionality for rendering debug visualizations of geometric shapes, such as points, lines, rays, spheres, AABBs, triangles, planes, and frustums.It includes the DebugShape and DebugDrawer classes, which facilitate the creation, customization, and rendering of debug shapes with various properties like color, transformation, and duration. Essential for visual debugging and graphical representations of spatial computations in the project.
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

#pragma once

#include "Math/Math.hpp"
#include "Shapes.hpp"

class Application;
class DebugDrawer;

//-----------------------------------------------------------------------------DebugShape
class DebugShape
{
public:
	DebugShape();

	// These functions allow chaining of setting properties (C++ closest thing to named parameters)

	// Set the color of this shape.
	DebugShape& Color(const Vector4& color);
	// Does this shape ignore depth test?
	DebugShape& OnTop(bool state);
	// How long should this shape draw for?
	DebugShape& Time(float time);
	// Set a random bit on the mask which can control what is drawn with the number keys.
	DebugShape& SetMaskBit(int bitIndex);
	// A transform to apply to the shape when drawing.
	// Primarily for spatial partitions in local space to draw their data.
	DebugShape& SetTransform(const Matrix4& transform);

private:
	friend DebugDrawer;

	Math::Vector4 mColor;
	float mTimer;
	unsigned int mMask;
	bool mOnTop;
	Matrix4 mTransform;

	// All of the lines use to draw this shape
	std::vector<LineSegment> mSegments;
};

//-----------------------------------------------------------------------------DebugDrawer
class DebugDrawer
{
public:
	DebugDrawer();

	void Update(float dt);
	void Draw();

	// Makes a new debug shape and inserts it into the shapes to draw.
	DebugShape& GetNewShape();


	// implemented functions
	DebugShape& DrawPoint(const Vector3& point);
	DebugShape& DrawLine(const LineSegment& line);
	DebugShape& DrawRay(const Ray& ray, float t);
	DebugShape& DrawSphere(const Sphere& sphere);
	DebugShape& DrawAabb(const Aabb& aabb);
	DebugShape& DrawTriangle(const Triangle& triangle);
	DebugShape& DrawPlane(const Plane& plane, float sizeX, float sizeY);
	DebugShape& DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3);
	DebugShape& DrawFrustum(const Frustum& frustum);
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
	DebugShape& DrawRay(DebugShape& shape, const Ray& ray, float t, float arrowHeadHeight=0.4f, int arrowLines=4);

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
	DebugShape& DrawDisc(DebugShape& shape, const Vector3& center,const Vector3&n, Vector3& v,Vector3&w, float radius, int segments = 20);
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
	DebugShape& DrawQuad(DebugShape& shape, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3);

private:
	friend Application;

	typedef std::vector<DebugShape> DebugShapeList;
	DebugShapeList mShapes;
	// Any active masks used to filter out what's currently drawing.
	unsigned int mActiveMask;

	Application* mApplication;
};

extern DebugDrawer* gDebugDrawer;
