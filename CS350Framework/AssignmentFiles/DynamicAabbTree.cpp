/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: DanymicAabbTree.cpp
Purpose: Implements the dynamic aabb tree
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

#include "Precompiled.hpp"
#include "Geometry.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;

void DynamicAabbTree::CastFrustumInternal(Node* node, const Frustum& frustum, CastResults& results) {
	if (node == nullptr) return;

	IntersectionType::Type intersection = FrustumAabb(frustum.GetPlanes(), node->mAabb.GetMin(), node->mAabb.GetMax(), node->mLastAxis);

	if (intersection == IntersectionType::Outside) {
		return; //node outside
	}
	else if (intersection == IntersectionType::Inside) {
		// node completely inside the frustum.
		AddAllLeaves(node, results);
	}
	else if (intersection == IntersectionType::Overlaps) {
		if (node->IsLeaf()) {
			results.AddResult(CastResult(node->mClientData));
		}
		else {
			CastFrustumInternal(node->mLeft, frustum, results);
			CastFrustumInternal(node->mRight, frustum, results);
		}
	}
}

void DynamicAabbTree::AddAllLeaves(Node* node, CastResults& results) {
	if (node == nullptr) return;

	if (node->IsLeaf()) {
		// add leaf node data to results
		results.AddResult(CastResult(node->mClientData));
	}
	else {
		// recursively add all leaves in the left and right subtrees
		AddAllLeaves(node->mLeft, results);
		AddAllLeaves(node->mRight, results);
	}
}

void DynamicAabbTree::Fatten(Aabb& aabb) {
	Vector3 center = aabb.GetCenter();
	Vector3 halfExtent = aabb.GetHalfSize();

	aabb.mMin = center - halfExtent * mFatteningFactor;
	aabb.mMax = center + halfExtent * mFatteningFactor;
}

DynamicAabbTree::DynamicAabbTree()
	:mRoot{ nullptr }
{
	mType = SpatialPartitionTypes::AabbTree;
}

void DynamicAabbTree::SplitNode(Node* nodeA, Node* nodeB, QueryResults& results) {
	// make "nodeA" to be the node with the larger (or equal) volume
	if (nodeB->mAabb.GetVolume() > nodeA->mAabb.GetVolume()) {
		std::swap(nodeA, nodeB);
	}

	if (nodeA->IsLeaf()==false) {
		SelfQuery(nodeA->mLeft, nodeB, results);
		SelfQuery(nodeA->mRight, nodeB, results);
	}
	else if (nodeB->IsLeaf()==false) {
		SelfQuery(nodeB->mLeft, nodeA, results);
		SelfQuery(nodeB->mRight, nodeA, results);
	}
}

void DynamicAabbTree::SelfQuery(Node* node, QueryResults& results) {
	if (!node || node->IsLeaf()) return;

	SelfQuery(node->mLeft, results);
	SelfQuery(node->mRight, results);
	SelfQuery(node->mLeft, node->mRight, results);
}

void DynamicAabbTree::SelfQuery(Node* nodeA, Node* nodeB, QueryResults& results) {
	if (nodeA == nullptr || nodeB == nullptr) return;

	if (AabbAabb(nodeA->mAabb.GetMin(), nodeA->mAabb.GetMax(), nodeB->mAabb.GetMin(), nodeB->mAabb.GetMax())) {
		if (nodeA->IsLeaf() && nodeB->IsLeaf()) {//leaf <-> leaf
			results.AddResult(QueryResult(nodeA->mClientData, nodeB->mClientData));
		}
		else {
			SplitNode(nodeA, nodeB, results);
		}
	}
}

DynamicAabbTree::~DynamicAabbTree()
{
	Clear(mRoot);
}

void DynamicAabbTree::Clear(Node* node) {
	if (node == nullptr) {
		return;
	}
	Clear(node->mLeft);
	Clear(node->mRight);
	delete node;
	node = nullptr;
}

void DynamicAabbTree::DebugDrawInternal(Node* node, int targetLevel, const Math::Matrix4& transform, const Vector4& color, int currentLevel){
	if (!node) return;

	// draw the current node's AABB if we're at the exact level, or if we're drawing all levels
	if (targetLevel == -1 || targetLevel == currentLevel) {
		gDebugDrawer->DrawAabb(node->mAabb).Color(color).SetTransform(transform);
	}

	//as long as current level is above the desired level, recursively draw the children
	if (targetLevel == -1 || currentLevel < targetLevel) {
		DebugDrawInternal(node->mLeft, targetLevel, transform, color, currentLevel + 1);
		DebugDrawInternal(node->mRight, targetLevel, transform, color, currentLevel + 1);
	}
}


void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	//fatten first.
	Fatten(data.mAabb);

	if (mRoot == nullptr) {
		mKey2Leaf[data.mClientData] = mRoot = new Node{ data };
		key.mVoidKey = data.mClientData;
		return;
	}

	Node* current = mRoot;
	current = FindBestChild(mRoot, data.mAabb);
	mKey2Leaf[current->mLeft->mClientData] = current->mLeft = current->Clone();
	mKey2Leaf[data.mClientData] = current->mRight = new Node{ data };//new one
	current->mRight->mParent = current->mLeft->mParent = current;
	key.mVoidKey = data.mClientData;
	BalanceFromNode(current);
}


void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) {
	auto toBeUpdated = mKey2Leaf.find(key.mVoidKey);
	if (toBeUpdated == mKey2Leaf.end()) {
		return; //key DNE
	}

	Node* node = toBeUpdated->second;
	// check if the new one entirely within the current one(or should compare with that of the parent?)
	if (node->mAabb.Contains(data.mAabb) == false) {
		RemoveData(key);
		InsertData(key, data); 
	}
	else {
		key.mVoidKey = data.mClientData;
	}
}


DynamicAabbTree::Node* DynamicAabbTree::FindBestChild(Node* currentNode, const Aabb& newAabb) {
	if (currentNode->IsLeaf()) {
		return currentNode;
	}

	Aabb combinedLeftAabb = Aabb::Combine(currentNode->mLeft->mAabb, newAabb);
	Aabb combinedRightAabb = Aabb::Combine(currentNode->mRight->mAabb, newAabb);

	float originalLeftSurfaceArea = currentNode->mLeft->mAabb.GetSurfaceArea();
	float originalRightSurfaceArea = currentNode->mRight->mAabb.GetSurfaceArea();

	float newLeftSurfaceArea = combinedLeftAabb.GetSurfaceArea();
	float newRightSurfaceArea = combinedRightAabb.GetSurfaceArea();

	float deltaLeftSurfaceArea = newLeftSurfaceArea - originalLeftSurfaceArea;
	float deltaRightSurfaceArea = newRightSurfaceArea - originalRightSurfaceArea;

	//go with the smaller area incr
	if (deltaLeftSurfaceArea < deltaRightSurfaceArea) {
		return FindBestChild(currentNode->mLeft, newAabb);
	}
	else {
		return FindBestChild(currentNode->mRight, newAabb);
	}
}


// Helper functions
bool DynamicAabbTree::IsUnbalanced(Node* node) {
	if (node == nullptr) return false;
	int leftHeight = node->mLeft ? static_cast<int>(node->mLeft->mHeight) : 0;
	int rightHeight = node->mRight ? static_cast<int>(node->mRight->mHeight) : 0;
	return std::abs(leftHeight - rightHeight) > 1;
}

DynamicAabbTree::Node* DynamicAabbTree::Rebalance(Node* oldParent) {
	//no internal node in this BVH is nullptr

	//1. identify nodes of interest
	bool isPivotLeftChild = oldParent->mLeft->mHeight > oldParent->mRight->mHeight;
	Node* pivot = isPivotLeftChild ? oldParent->mLeft : oldParent->mRight;
	bool isSmallerLeftChildOfPivot = pivot->mLeft->mHeight < pivot->mRight->mHeight;
	Node* largerChildOfPivot = isSmallerLeftChildOfPivot ? pivot->mRight : pivot->mLeft;
	Node* smallelrChildOfPivot = isSmallerLeftChildOfPivot ? pivot->mLeft : pivot->mRight;

	//2. detach connections
	//detach-(1) : old parent -> pivot -> smaller
	smallelrChildOfPivot->mParent = pivot->mParent = nullptr;
	//detach-(2) : old parent <- pivot <- smaller
	(isPivotLeftChild ? oldParent->mLeft : oldParent->mRight) = nullptr;
	(isSmallerLeftChildOfPivot ? pivot->mLeft : pivot->mRight) = nullptr;

	//3. reconnect
	//(1) grand parent <-> pivot
	Node* grandParent = oldParent->mParent;
	if (grandParent) {
		bool isOldParentLeftChild = grandParent->mLeft == oldParent;
		(isOldParentLeftChild ? grandParent->mLeft : grandParent->mRight) = pivot;
		pivot->mParent = grandParent;
	}
	//(2) parent <-> old parent
	(isSmallerLeftChildOfPivot ? pivot->mLeft : pivot->mRight) = oldParent;
	oldParent->mParent = pivot;
	//(3) old parent <-> smaller
	(isPivotLeftChild ? oldParent->mLeft : oldParent->mRight) = smallelrChildOfPivot;
	smallelrChildOfPivot->mParent = oldParent;

	if (oldParent == mRoot) {
		mRoot = pivot;
	}
	return oldParent;
}

void DynamicAabbTree::UpdateAabb(Node* node) {
	if (node == nullptr || node->IsLeaf()) return;
	node->mAabb = Aabb::Combine(node->mLeft ? node->mLeft->mAabb : Aabb(), node->mRight ? node->mRight->mAabb : Aabb());
}

void DynamicAabbTree::PreOrderFillout(std::vector<SpatialPartitionQueryData>& results, Node* node, int depth) const{
	if (!node) return;

	SpatialPartitionQueryData data(SpatialPartitionData{ node->mClientData,node->mAabb });
	data.mDepth = depth;
	results.push_back(data);
	PreOrderFillout(results, node->mLeft, depth + 1);
	PreOrderFillout(results, node->mRight, depth + 1);
}


void DynamicAabbTree::UpdateHeight(Node* node) {
	if (node == nullptr) return;
	size_t leftHeight = node->mLeft ? node->mLeft->mHeight : 0;
	size_t rightHeight = node->mRight ? node->mRight->mHeight : 0;
	node->mHeight = 1 + std::max(leftHeight, rightHeight);
}

void DynamicAabbTree::BalanceFromNode(Node* node) {
	while (node != nullptr) {
		UpdateHeight(node);
		UpdateAabb(node);
		node = IsUnbalanced(node) ? Rebalance(node) : node->mParent;
	}
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key) {
	auto toBeDeleted = mKey2Leaf.find(key.mVoidKey);
	if (toBeDeleted == mKey2Leaf.end()) {
		return; //key DNE
	}

	Node* node = toBeDeleted->second;
	Node* parent = node->mParent;
	Node* grandParent{ nullptr };
	Node* sibling{ nullptr };
	if (parent) {
		grandParent = parent->mParent;
		bool isDeleteNodeLeftChild = parent->mLeft == node;
		sibling = (isDeleteNodeLeftChild ? parent->mRight : parent->mLeft);
		if (grandParent) {
			bool isParentLeftChild = grandParent->mLeft == parent;
			//connect, GrandParent -> sibling
			(isParentLeftChild ? grandParent->mLeft : grandParent->mRight) = sibling;
		}
		//connect, GrandParent <- sibling
		sibling->mParent = grandParent;
		if (parent == mRoot) {
			mRoot = sibling;//sibling is now the root
		}
		node->mParent = parent->mLeft = parent->mRight = parent->mParent = nullptr;//to prevent dangling references 
		delete parent;
	}

	delete node;
	if (node == mRoot) {
		mRoot = nullptr;
	}

	mKey2Leaf.erase(key.mVoidKey);
	BalanceFromNode(grandParent);
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask){
	DebugDrawInternal(mRoot, level, transform, color);
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results) {
	if (!mRoot) return;

	float tMin{};
	//if the ray intersects with the root, then proceed with traversal
	if (RayAabb(ray.mStart, ray.mDirection, mRoot->mAabb.mMin, mRoot->mAabb.mMax, tMin)) {
		CastRayInternal(mRoot, ray, results, tMin);
	}
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
	if (!mRoot) return;
	CastFrustumInternal(mRoot, frustum, results);
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
	if (mRoot == nullptr) return;
	SelfQuery(mRoot, results);
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	PreOrderFillout(results, mRoot);
}

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
std::pair<float, DynamicAabbTree::RayHitType>
DynamicAabbTree::CastRayInternal(Node* node, const Ray& ray, CastResults& results, float tMin) {
	if (!node) return { std::numeric_limits<float>::infinity(), RayHitType::NoHit };

	// leaf-hit
	if (node->IsLeaf()) {
		results.AddResult({ node->mClientData, tMin });
		return { tMin, RayHitType::LeafHit };
	}

	// check the children
	float tMinLeft = std::numeric_limits<float>::infinity(), tMinRight = std::numeric_limits<float>::infinity();
	bool hitLeft = RayAabb(ray.mStart, ray.mDirection, node->mLeft->mAabb.mMin, node->mLeft->mAabb.mMax, tMinLeft);
	bool hitRight = RayAabb(ray.mStart, ray.mDirection, node->mRight->mAabb.mMin, node->mRight->mAabb.mMax, tMinRight);

	// determine which child to traverse first based on hit times, and then traverse the second child if we needed
	std::pair<float, DynamicAabbTree::RayHitType> hitResultFirst{ std::numeric_limits<float>::infinity(), RayHitType::NoHit };
	std::pair<float, DynamicAabbTree::RayHitType> hitResultSecond{ std::numeric_limits<float>::infinity(), RayHitType::NoHit };

	if (hitLeft || hitRight) {
		if (hitLeft && (!hitRight || tMinLeft <= tMinRight)) {
			hitResultFirst = CastRayInternal(node->mLeft, ray, results, tMinLeft);
			if (hitRight) hitResultSecond = CastRayInternal(node->mRight, ray, results, tMinRight);
		}
		else if (hitRight) {
			hitResultFirst = CastRayInternal(node->mRight, ray, results, tMinRight);
			if (hitLeft) hitResultSecond = CastRayInternal(node->mLeft, ray, results, tMinLeft);
		}
	}

	//return the closer hit
	if (hitResultFirst.first <= hitResultSecond.first || hitResultSecond.second == RayHitType::NoHit) {
		return hitResultFirst;
	}
	else {
		return hitResultSecond;
	}
}

DynamicAabbTree::Node* DynamicAabbTree::Node::Clone() {
	Node* clone = new Node();
	clone->mAabb = mAabb;
	clone->mClientData = mClientData;
	mClientData = nullptr;
	clone->mLeft = clone->mRight = clone->mParent = nullptr;
	clone->mHeight = mHeight;
	clone->mLastAxis = mLastAxis;
	return clone;
}
