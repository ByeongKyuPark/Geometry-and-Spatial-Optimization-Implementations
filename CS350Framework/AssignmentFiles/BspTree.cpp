/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: BspTree.cpp
Purpose: Defines the BSP tree
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
          OpenGL Info: 4.6.0 - Build 31.0.101.3413
          Version: 4.6.0 - Build 31.0.101.3413
          Vendor: Intel
          Renderer: Intel(R) UHD Graphics
Project: byeonggyu.park_CS350_4
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 4/3/2024
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

using std::begin;
using std::end;

BspTreeQueryData::BspTreeQueryData()
{
  mDepth = 0;
}

void BspTree::ConstructTrianglesFromVertices(TriangleList& list, Vector3 vertices[], int numVertices) {
    list.push_back(Triangle(vertices[0], vertices[1], vertices[2]));
    if (numVertices == 4) { // split the quad into two triangles and add the later one
        list.push_back(Triangle(vertices[0], vertices[2], vertices[3]));
    }
}

void BspTree::RayCastRecursive(const std::shared_ptr<BspNode>& node, const Ray& ray, float& tHit, float tMin, float tMax, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
    if (node == nullptr) return;

    float tPlane{Math::PositiveMax()};
    Plane& plane = node->mSplitPlane;
    IntersectionType::Type rayStartIntersectionType = PointPlane(ray.mStart, plane.mData, planeThicknessEpsilon);

    //by default, near is inside of the split plane
    std::shared_ptr<BspNode> nearSide{node->mFront};
    std::shared_ptr<BspNode> farSide{node->mBack};

    //if ray_start belongs to the outside, then simply swap them
    if (rayStartIntersectionType == IntersectionType::Outside) {
        nearSide.swap(farSide);//shared_ptr::swap
    }

    // edge case (1)
    if (rayStartIntersectionType==IntersectionType::Coplanar){
        RayCastNearCoplanarFar(nearSide,farSide,node->mCoplanarTriangles,ray,tHit,tMin,tMax,tMin,tMax,planeThicknessEpsilon,triExpansionEpsilon,debuggingIndex);
        return;
    }

    // edge case (2)
    if (RayPlane(ray.mStart, ray.mDirection, plane.mData, tPlane) == false){
        RayCastRecursive(nearSide, ray, tHit, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
        return;
    }

    float tE = Math::Abs(planeThicknessEpsilon / Math::Dot(plane.GetNormal(), ray.mDirection.Normalized()));

    // Case A
    if (tPlane < 0.f){
        RayCastRecursive(nearSide, ray, tHit, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    }
    // Case B
    else if (0 < tPlane && tPlane < tMin){
        RayCastRecursive(farSide, ray, tHit, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    }
    // Case C
    if (tMin - tE <= tPlane && tPlane <= tMax + tE){
        RayCastNearCoplanarFar(
            nearSide, 
            farSide, 
            node->mCoplanarTriangles, 
            ray, 
            tHit,                   
            tMin        , tPlane + tE,              // near range 
            tPlane - tE , tMax       ,              //  far range
            planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex // epsilons...
        );
    }
    // Case D
    else if (tMax < tPlane){
        RayCastRecursive(nearSide, ray, tHit, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    }
}

void BspTree::RayCastNearCoplanarFar(const std::shared_ptr<BspNode>& nearSide, const std::shared_ptr<BspNode>& farSide, const TriangleList& coplanarTriangles, const Ray& ray, float& tHit, float tMinNear, float tMaxNear, float tMinFar, float tMaxFar, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
    //near
    RayCastRecursive(nearSide, ray, tHit, tMinNear, tMaxNear, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
    //coplanar
    for (const Triangle& tri : coplanarTriangles) {
        float tTri{ Math::PositiveMax() };
        if (RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], tTri, triExpansionEpsilon)) {
            tHit = std::min(tTri, tHit);
        }
    }
    //far
    RayCastRecursive(farSide, ray, tHit, tMinFar, tMaxFar, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
}

void BspTree::ConstructRecursive(const std::shared_ptr<BspNode>& node, const TriangleList& triangles, float k, float epsilon) {
    if (triangles.empty()) return;

    size_t bestIndex = PickSplitPlane(triangles, k, epsilon);
    if (bestIndex == std::numeric_limits<size_t>::max()) {
        node->mCoplanarTriangles = triangles; // no valid split plane found
        return;
    }

    TriangleList frontList, backList, coplanarList;
    Plane splitPlane = Plane(triangles[bestIndex].mPoints[0], triangles[bestIndex].mPoints[1], triangles[bestIndex].mPoints[2]);
    node->mSplitPlane = splitPlane;

    for (size_t i = 0; i < triangles.size(); ++i) {
        if (i == bestIndex) {
            coplanarList.push_back(triangles[i]); // coplanar splitting plane
            continue;
        }

        float t[3]{Math::PositiveMax(),};
        IntersectionType::Type classifications[3];
        IntersectionType::Type type = PlaneTriangleWithIntersections(splitPlane.mData, triangles[i].mPoints[0], triangles[i].mPoints[1], triangles[i].mPoints[2], t, classifications, epsilon, false);

        switch (type) {
        case IntersectionType::Inside:
            frontList.push_back(triangles[i]);
            break;
        case IntersectionType::Outside:
            backList.push_back(triangles[i]);
            break;
        case IntersectionType::Overlaps:
            SplitTriangle(splitPlane, triangles[i], coplanarList, coplanarList, frontList, backList, epsilon);
            break;
        case IntersectionType::Coplanar:
            coplanarList.push_back(triangles[i]);
            break;
        }
    }

    node->mCoplanarTriangles = coplanarList; // store coplanar triangles in the current node
    if (!frontList.empty()) {
        node->mFront = std::make_shared<BspNode>();
        ConstructRecursive(node->mFront, frontList, k, epsilon);
    }
    if (!backList.empty()) {
        node->mBack = std::make_shared<BspNode>();
        ConstructRecursive(node->mBack, backList, k, epsilon);
    }
}


void BspTree::PreOrderFillout(std::vector<BspTreeQueryData>& results, BspNode* node, int depth) const {
    if (!node) return;

    BspTreeQueryData data;
    data.mTriangles = node->mCoplanarTriangles;
    data.mDepth = depth;
    results.push_back(data);
    PreOrderFillout(results, node->mFront.get(), depth + 1);
    PreOrderFillout(results, node->mBack.get(), depth + 1);
}

void BspTree::PreOrderGetTriangles(const std::shared_ptr<BspNode>& node, TriangleList& triangles) const {
    if (!node) return;

    //(1) coplanar
    node->GetTriangles(triangles);
    //(2) left child (front)
    if (node->mFront) PreOrderGetTriangles(node->mFront, triangles);
    //(3) right child (back)
    if (node->mBack) PreOrderGetTriangles(node->mBack, triangles);
}

void BspTree::PreOrderInvert(std::shared_ptr<BspNode>& node) {
    if (!node) return;
    //(1) flip split plane
    node->mSplitPlane.mData = -node->mSplitPlane.mData; //negation is cheaper than multiplication..

    //(2) flip coplanar polygons
    for (Triangle& tri : node->mCoplanarTriangles) {
        std::swap(tri.mPoints[0], tri.mPoints[1]);
    }

    //(3) swap Front <-> Back
    std::swap(node->mFront, node->mBack);

    //traverse back first as back & front were swapped (doeesn't matter)
    PreOrderInvert(node->mBack);
    PreOrderInvert(node->mFront);
}

void BspTree::DebugDrawRecursive(const std::shared_ptr<BspNode>& node, int currentLevel, int targetLevel, const Vector4& color, int bitMask) const {
    if (!node) return;

    // at the target level or (targets are all levels), draw the triangles
    if (targetLevel == -1 || currentLevel == targetLevel) {
        TriangleList triangles;
        node->GetTriangles(triangles);
        for (const auto& tri : triangles) {
            tri.DebugDraw().Color(color).SetMaskBit(bitMask);
        }
    }

    // still hasn't reached the target level yet. goes into deeper
    if (targetLevel == -1 || currentLevel < targetLevel) {
        DebugDrawRecursive(node->mFront, currentLevel + 1, targetLevel, color, bitMask);
        DebugDrawRecursive(node->mBack, currentLevel + 1, targetLevel, color, bitMask);
    }
}


/******Student:Assignment4******/
//  Reused the 'PlaneTriangle' function from Assignment 1, enhancing it with `PlaneTriangleWithIntersections()` that includes an `earlyExit` argument.
// This argument, when set to 'false', ensures all intersections are calculated for accurately determining the triangle's position relative to the plane, 
void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri,
    TriangleList& coplanarFront, TriangleList& coplanarBack,
    TriangleList& front, TriangleList& back, float epsilon) {
    float t[3] = { -1.f, -1.f, -1.f }; // intersection parameters for each edge
    IntersectionType::Type types[3]{IntersectionType::NotImplemented,}; // classification for each vertex

    Vector3 frontVerts[4]{}, backVerts[4]{}; // storage for vertices potentially forming quads
    int frontCount{}, backCount{};

    IntersectionType::Type overallClassification = PlaneTriangleWithIntersections(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], t, types, epsilon, false);

    // non-straddling cases
    if (overallClassification == IntersectionType::Coplanar) {
        // calc the triangle's normal to determine its orientation relative to the plane
        Vector3 triNormal = Cross(tri.mPoints[1] - tri.mPoints[0], tri.mPoints[2] - tri.mPoints[0]);
        triNormal.Normalize();
        
        // determine if the triangle is facing the same direction as the plane normal
        Dot(triNormal, plane.GetNormal()) > 0 ? coplanarFront.push_back(tri) : coplanarBack.push_back(tri);
        return;
    }
    else if (overallClassification == IntersectionType::Inside) {
        front.push_back(tri);
        return;
    }
    else if (overallClassification == IntersectionType::Outside) {
        back.push_back(tri);
        return;
    }

    // process each edge based on the 9-cases
    for (int i{}; i < 3; ++i) {
        int next = (i + 1) % 3;
        const Vector3& A = tri.mPoints[i];
        const Vector3& B = tri.mPoints[next];
        if (t[i] >= 0.f && t[i] <= 1.f) { // intersection !
            Vector3 intersection = A + t[i]*(B - A);

            if ((types[i] == IntersectionType::Inside || types[i] == IntersectionType::Coplanar) && types[next] == IntersectionType::Outside) {
                frontVerts[frontCount++] = intersection;
                backVerts[backCount++] = intersection;
                backVerts[backCount++] = B;
            }
            else if (types[i] == IntersectionType::Outside && (types[next] == IntersectionType::Inside || types[next] == IntersectionType::Coplanar)) {
                backVerts[backCount++] = intersection;
                frontVerts[frontCount++] = intersection;
                frontVerts[frontCount++] = B;
            }
        }
        else { // no direct intersection, proceed based on vertex positions
            if (types[next] == IntersectionType::Inside || types[next] == IntersectionType::Coplanar) {
                frontVerts[frontCount++] = B;
            }
            if (types[next] == IntersectionType::Outside || types[next] == IntersectionType::Coplanar) {
                backVerts[backCount++] = B;
            }
        }
    }

    // splitting quads into triangles (if necessary)
    ConstructTrianglesFromVertices(front, frontVerts, frontCount);
    ConstructTrianglesFromVertices(back, backVerts, backCount);
}

/******Student:Assignment4******/
float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon){

    Vector3 v1 = triangles[testIndex].mPoints[1] - triangles[testIndex].mPoints[0];
    Vector3 v2 = triangles[testIndex].mPoints[2] - triangles[testIndex].mPoints[0];
    Vector3 normal = v1.Cross(v2);

    //triangle index out or range OR a degenerated triangle
    if (testIndex >= triangles.size() || normal.Length() < epsilon) {
        return Math::PositiveMax();
    }

    const Plane TestPlane{ normal,triangles[testIndex].mPoints[0]};

    int numStraddling{}, numFront{}, numBehind{};
    for (size_t i{}; i < triangles.size(); ++i) {
        if (i == testIndex) continue; // skip colinear

        IntersectionType::Type classification = PlaneTriangle(TestPlane.mData, triangles[i].mPoints[0], triangles[i].mPoints[1], triangles[i].mPoints[2],epsilon);
        switch (classification) {
        case IntersectionType::Inside:
            numFront++;
            break;
        case IntersectionType::Outside:
            numBehind++;
            break;
        case IntersectionType::Overlaps:
            numStraddling++;
            break;
        default: // coplanar triangles are not counted
            break;
        }
    }

    return k * numStraddling + (1.f-k) * Math::Abs(numFront - numBehind);
}

/******Student:Assignment4******/
size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon){
    float lowestScore = Math::PositiveMax();
    size_t bestIndex = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < triangles.size(); ++i) {
        float score = CalculateScore(triangles, i, k, epsilon);
        if (score < lowestScore) {
            lowestScore = score;
            bestIndex = i;
        }
    }

    return bestIndex;
}

/******Student:Assignment4******/
void BspTree::Construct(const TriangleList& triangles, float k, float epsilon){
    mRoot = std::make_shared<BspNode>();
    ConstructRecursive(mRoot, triangles, k, epsilon);
}

/******Student:Assignment4******/
bool BspTree::RayCast(const Ray& ray, float& hitT, float planeEpsilon, float triExpansionEpsilon, int debuggingIndex) {
    RayCastRecursive(mRoot, ray, hitT, 0.f, Math::PositiveMax(), planeEpsilon, triExpansionEpsilon,debuggingIndex);
    return hitT < Math::PositiveMax();//returns "True" as long as the ray hits something
}


/******Student:Assignment4******/
void BspTree::AllTriangles(TriangleList& triangles) const {
    PreOrderGetTriangles(mRoot, triangles);
}

/******Student:Assignment4******/
void BspTree::Invert()
{
    PreOrderInvert(mRoot);
}

/******Student:Assignment4******/
void BspTree::ClipTo(BspTree* tree, float epsilon)
{
    mRoot->ClipTo(tree->mRoot, epsilon);
}

/******Student:Assignment4******/
void BspTree::Union(BspTree* rhs, float k, float epsilon)
{
    std::shared_ptr<BspNode>& nodeA = this->mRoot;
    std::shared_ptr<BspNode>& nodeB = rhs->mRoot;

    nodeA->ClipTo(nodeB, epsilon);
    nodeB->ClipTo(nodeA, epsilon);

    // Remove coplanar faces (from the CSG.pdf)
    rhs->Invert();
    nodeB->ClipTo(nodeA, epsilon);
    rhs->Invert();

    TriangleList results;
    this->AllTriangles(results);
    rhs->AllTriangles(results);

    Construct(results, k, epsilon);
}

/******Student:Assignment4******/
void BspTree::Intersection(BspTree* rhs, float k, float epsilon)
{
    this->Invert();         // ~A
    rhs->Invert();          // ~B
    Union(rhs, k, epsilon); // ~A U ~B
    Invert();               // ~(~A U ~B)
}

/******Student:Assignment4******/
void BspTree::Subtract(BspTree* rhs, float k, float epsilon)
{
    rhs->Invert();                  //~B
    Intersection(rhs, k, epsilon);  //A N (~B) = A-B
}

/******Student:Assignment4******/
void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
    PreOrderFillout(results, mRoot.get());
}

/******Student:Assignment4******/
void BspTree::DebugDraw(int level, const Vector4& color, int bitMask){
    DebugDrawRecursive(mRoot, 0, level, color, bitMask);
}

void BspTree::BspNode::GetTriangles(TriangleList& triangles) const
{
    if (mCoplanarTriangles.empty()) return;
    triangles.insert(end(triangles), begin(mCoplanarTriangles), end(mCoplanarTriangles));
}

void BspTree::BspNode::ClipTo(std::shared_ptr<BspNode>& scissorNode, float epsilon)
{
    if (!scissorNode) return;

    scissorNode->ClipTriangles(mCoplanarTriangles, epsilon);

    if (mFront) mFront->ClipTo(scissorNode, epsilon);
    if (mBack) mBack->ClipTo(scissorNode, epsilon);
}

// clips a triangle against this node's plane without altering the existing tree structure.
// fragments are stored immediately, avoiding modifications to the current tree.
void BspTree::BspNode::ClipTriangle(const Triangle& triangleToSplit, TriangleList& results, float epsilon) const
{
    TriangleList outFragments; // fragments lying in front of the split plane
    TriangleList inFragments;  // fragments lying behind the split plane
    
    //split the 'triangleToSplit' using the split plane of the current node
    BspTree::SplitTriangle(mSplitPlane, triangleToSplit, outFragments, inFragments, outFragments, inFragments, epsilon);

    //(1) processes fragments in front, potentially splitting them further
    ProcessFragments(outFragments, results, epsilon, true);

    //(2) processes fragments behind, potentially splitting them further
    ProcessFragments(inFragments, results, epsilon, false);
}

void BspTree::BspNode::ClipTriangles(TriangleList& triangles, float epsilon) const
{
    TriangleList result;
    std::for_each(begin(triangles),end(triangles), [&](Triangle& tri) {
        ClipTriangle(tri, result, epsilon); 
        });
    triangles = result;
}

void BspTree::BspNode::ProcessFragments(const TriangleList& fragments, TriangleList& results, float epsilon, bool isFront) const {
    const std::shared_ptr<BspNode>& side = isFront ? mFront : mBack; //relevant side

    if (!side) {//reached a "Empty" leaf.  note nullptr means "Empty" leaf
        if(isFront){
            // directly stores fragments on the front side in the results, bypassing intermediate storage and retrieval
            results.insert(end(results), begin(fragments), end(fragments));
        }
        // discard fragments on the back side ("Solid" leaf)
    }
    else {//internal node. need to traverse further.
        std::for_each(begin(fragments), end(fragments), [&](const Triangle& tri) {
            side->ClipTriangle(tri, results, epsilon);
            });
    }
}
