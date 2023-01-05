// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LIDAR__LIDARFRAMEFEATURES_HPP_
#define ROMEA_CORE_LIDAR__LIDARFRAMEFEATURES_HPP_

// std
#include <memory>
#include <vector>

// romea
#include "romea_core_common/pointset/PointSet.hpp"
#include "romea_core_common/pointset/NormalSet.hpp"
#include "romea_core_common/pointset/KdTree.hpp"

namespace romea
{

template<class PointType>
struct LIDARFrameFeatures
{
  using Ptr = std::shared_ptr<LIDARFrameFeatures<PointType>>;
  using ConstPtr = std::shared_ptr<const LIDARFrameFeatures<PointType>>;

  KdTree<PointType> kdTree;
  NormalSet<PointType> normals;
  std::vector<double> curvatures;
  std::vector<double> pointsNormalsReliability;
};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDARFRAMEFEATURES_HPP_
