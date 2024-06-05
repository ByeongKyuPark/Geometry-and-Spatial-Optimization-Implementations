///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: SimpleNSquared.cpp
Purpose: Implements the functionalities for N-Squred Spatial partition
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

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
	mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// Doing this lazily (and bad, but it's n-squared...).
	// Just store as the key what the client data is so we can look it up later.
	key.mVoidKey = data.mClientData;
	mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
	// Find the key data and remove it
	for (size_t i = 0; i < mData.size(); ++i)
	{
		if (mData[i] == key.mVoidKey)
		{
			mData[i] = mData.back();
			mData.pop_back();
			break;
		}
	}
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
	// Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
	// Add everything
	for (size_t i = 0; i < mData.size(); ++i)
	{
		CastResult result;
		result.mClientData = mData[i];
		results.AddResult(result);
	}
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
	// Add everything
	for (size_t i = 0; i < mData.size(); ++i)
	{
		CastResult result;
		result.mClientData = mData[i];
		results.AddResult(result);
	}
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
	// Add everything
	for (size_t i = 0; i < mData.size(); ++i)
	{
		for (size_t j = i + 1; j < mData.size(); ++j)
		{
			results.AddResult(QueryResult(mData[i], mData[j]));
		}
	}
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
	data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	for (size_t i = 0; i < mData.size(); ++i)
	{
		SpatialPartitionQueryData data;
		data.mClientData = mData[i];
		results.push_back(data);
	}
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
	mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	//does not assume anything about SpatialPartitionData::mClientData, just store and return
	mDataMap[data.mClientData] = data;
	key.mVoidKey = data.mClientData;
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	//Update is O(1)
	mDataMap[key.mVoidKey] = data;//if key does not exist, unordred_map creates a new one
}
void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
	//Removal is O(1)
	mDataMap.erase(key.mVoidKey); //does nothing if the key is not found
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{											
	for (auto& clientData : mDataMap) {
		// fetch the SpatialPartitionData
		SpatialPartitionData& data = clientData.second;

		//use the passed in transform, color, and bit mask
		gDebugDrawer->DrawSphere(data.mBoundingSphere).SetTransform(transform).Color(color).SetMaskBit(bitMask);
	}
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
	for (auto& entry : mDataMap) {
		float t{};
		if (RaySphere(ray.mStart, ray.mDirection, entry.second.mBoundingSphere.GetCenter(), entry.second.mBoundingSphere.GetRadius(), t)) {
			// Add the object and its intersection distance 't' to the results. The distance 't' can be used to sort
			// the results, with objects having smaller 't' values colliding first with the ray.
			results.AddResult(CastResult(entry.first,t));
		}
	}
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
	size_t lastAxis{};
	for (auto& entry : mDataMap) {
		if (FrustumSphere(frustum.GetPlanes(), entry.second.mBoundingSphere.GetCenter(), entry.second.mBoundingSphere.GetRadius(), lastAxis) != IntersectionType::Outside) {
			results.AddResult(CastResult(entry.first));
		}
	}
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
	for (auto it1 = mDataMap.begin(); it1 != mDataMap.end(); ++it1) {
		for (auto it2 = std::next(it1); it2 != mDataMap.end(); ++it2) {
			if (SphereSphere(it1->second.mBoundingSphere.GetCenter(), it1->second.mBoundingSphere.GetRadius(),
				it2->second.mBoundingSphere.GetCenter(), it2->second.mBoundingSphere.GetRadius())) {
				results.AddResult(QueryResult(it1->first, it2->first));
			}
		}
	}
}
void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	//since 'RemoveData' ensures that mDataMap contains only valid entries,
	//no need to check validity here
	for (const auto& entry : mDataMap) {
		SpatialPartitionQueryData data(entry.second);
		results.push_back(data);
	}
}
