// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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
namespace core
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

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDARFRAMEFEATURES_HPP_
