/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: DanymicAabbTree.hpp
Purpose: Defines the dynamic aabb tree
Language: C++14 MSVC
Platform: MSVC VS2022(17.4.4), Windows11 Home (x64, 23H2)  //tested on Windows 10 Home as well
          OpenGL Info: 4.6.0 - Build 31.0.101.3413
          Version: 4.6.0 - Build 31.0.101.3413
          Vendor: Intel
          Renderer: Intel(R) UHD Graphics
Project: byeonggyu.park_CS350_3
Author: ByeongKyu Park(byeonggyu.park), 0055254
Creation date: 3/6/2024
End Header -------------------------------------------------------*/

#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"
#include <unordered_map>

/******Student:Assignment3******/
class DynamicAabbTree : public SpatialPartition
{
    // implementations here
    struct Node {
        Aabb mAabb;
        void* mClientData;
        Node* mLeft, * mRight, * mParent;
        size_t mHeight;
        size_t mLastAxis;
        bool IsLeaf() const { return mHeight == 0; }

        Node() :mAabb{}, mClientData{nullptr}, mLeft{nullptr}, mRight{nullptr}, mParent{nullptr}, mHeight{}, mLastAxis{} 
        {}
        //note 
        Node(Node* rhs) : mAabb(rhs->mAabb), mClientData{ rhs->mClientData }, mLeft{ nullptr }, mRight{ nullptr }, mParent{ nullptr }, mHeight{}, mLastAxis{} {
            rhs->mClientData = nullptr;//reset the client data
        }
        Node(const SpatialPartitionData& data) : mAabb(data.mAabb), mClientData{ data.mClientData }, mLeft{ nullptr }, mRight{ nullptr }, mParent{ nullptr }, mHeight{}, mLastAxis{}
        {}

        Node* Clone();

    };
    enum class RayHitType {HitSomething, LeafHit, NoHit};

    bool IsUnbalanced(Node* node);
    void Fatten(Aabb& aabb);

    Node* Rebalance(Node* oldParent);
    Node* FindBestChild(Node* node, const Aabb& aabb);

    void UpdateHeight(Node* node);
    void UpdateAabb(Node* node);
    void BalanceFromNode(Node* node);

    /**
     * Casts a ray through the AABB tree to find the closest hit node. Recursively checks each node, prioritizing closer hits.
     *
     * @param node The current node being checked.
     * @param ray The ray being cast through the tree.
     * @param results Stores the results of ray casts that hit.
     * @param tMin The minimum hit time found so far; used to determine the closest hit.
     * @param rayAabbChecked Flag indicating if the current node's AABB intersection has already been checked.
     * @return A pair containing the minimum hit time and the type of hit (leaf, hit something, or no hit).
     */
    std::pair<float, RayHitType> 
    CastRayInternal(Node* node, const Ray& ray, CastResults& results, float tMin = std::numeric_limits<float>::infinity());
    
    void CastFrustumInternal(Node* node, const Frustum& frustum, CastResults& results);
    void AddAllLeaves(Node* node, CastResults& results);

    void SelfQuery(Node* node, QueryResults& results);
    void SelfQuery(Node* nodeA, Node* nodeB, QueryResults& results);
    void SplitNode(Node* nodeA, Node* nodeB, QueryResults& results);

    void PreOrderFillout(std::vector<SpatialPartitionQueryData>& results, Node* node, int depth = 0) const;
    void Clear(Node* node); // utility to clear the tree recursively
    void DebugDrawInternal(Node* node, int level, const Math::Matrix4& transform, const Vector4& color, int currentLevel = 0);
 
    Node* mRoot;
    std::unordered_map<void*, Node*> mKey2Leaf;
public:
    DynamicAabbTree();
    ~DynamicAabbTree();

    // Spatial Partition Interface
    void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
    void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
    void RemoveData(SpatialPartitionKey& key) override;

    void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

    void CastRay(const Ray& ray, CastResults& results) override;
    void CastFrustum(const Frustum& frustum, CastResults& results) override;

    void SelfQuery(QueryResults& results) override;

    void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

    static const float mFatteningFactor;
};
