///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: SimpleNSquared.cpp
Purpose: Defines the functionalities for N-Squred Spatial partition
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
#pragma once

#include "SpatialPartition.hpp"
#include <unordered_map>//added

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
// A very bad, brute force spatial partition that is used for assignment 1 (before you get to implement something better).
// Do not implement your spatial partitions this way, make sure that update and remove are O(1) (no searches).
class NSquaredSpatialPartition : public SpatialPartition
{
public:
  NSquaredSpatialPartition();

  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const override;
  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  std::vector<void*> mData;
};

/******Student:Assignment2******/
// Implement the n-squared sphere spatial partition
//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
class BoundingSphereSpatialPartition : public SpatialPartition
{
public:
  BoundingSphereSpatialPartition();

  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  // here, directly used the client data pointer as the key in the map, not making any assumptions about the type(even value) of the client data
  std::unordered_map<void*, SpatialPartitionData> mDataMap; 
};
