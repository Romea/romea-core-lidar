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


// std
#include <algorithm>

// romea
#include "romea_core_lidar/LIDAR2DFrameBuilder.hpp"
#include "romea_core_common/pointset/PointTraits.hpp"

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::LIDAR2DFrameBuilder()
: LIDAR2DFrameBuilderBase<PointScalarType>()
{
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::LIDAR2DFrameBuilder(const LIDAR2D & lidar)
: LIDAR2DFrameBuilderBase<PointScalarType>(lidar)
{
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
PointSet<PointType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::createFrame(const RangeVector & ranges)const
{
  return createFrame(
    ranges,
    this->minimalRange_,
    this->maximalRange_,
    0,
    this->rayUnitVectors_.size() - 1);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
PointSet<PointType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::createFrame(
  const RangeVector & ranges,
  const double & userMinimalRange,
  const double & userMaximalRange)const
{
  return createFrame(
    ranges,
    userMinimalRange,
    userMaximalRange,
    0,
    this->rayUnitVectors_.size() - 1);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
PointSet<PointType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::createFrame(
  const RangeVector & ranges,
  const double & userMinimalRange,
  const double & userMaximalRange,
  const size_t & userFirstAzimutAngleIndex,
  const size_t & userLastAzimutAngleIndex)const
{
  assert(userFirstAzimutAngleIndex < this->rayUnitVectors_.size());
  assert(userLastAzimutAngleIndex < this->rayUnitVectors_.size());

  typename LIDAR2DFrameBuilderBase<PointScalarType>::Vector4 point;
  double minimalRange = std::max(this->minimalRange_, userMinimalRange);
  double maximalRange = std::min(this->maximalRange_, userMaximalRange);

  PointSet<PointType> points;
  points.reserve(userLastAzimutAngleIndex - userFirstAzimutAngleIndex);
  for (size_t n = userFirstAzimutAngleIndex; n <= userLastAzimutAngleIndex; n++) {
    double d = ranges[n];
    if (d > minimalRange && d < maximalRange) {
      point = this->H_ * (this->rayUnitVectors_[n] * ranges[n] + this->Zero_);
      points.emplace_back(point.template segment<PointTraits<PointType>::SIZE>(0));
    }
  }

  return points;
}


//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
PointSet<PointType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::createFrame(
  const RangeVector & ranges,
  const PointRange & pointRange)const
{
  return createFrame(
    ranges,
    pointRange,
    0,
    this->rayUnitVectors_.size() - 1);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
PointSet<PointType>
LIDAR2DFrameBuilder<PointType, RangeScalarType>::createFrame(
  const RangeVector & ranges,
  const PointRange & pointRange,
  const size_t & userFirstAzimutAngleIndex,
  const size_t & userLastAzimutAngleIndex)const
{
  assert(userFirstAzimutAngleIndex < this->rayUnitVectors_.size());
  assert(userLastAzimutAngleIndex < this->rayUnitVectors_.size());

  typename LIDAR2DFrameBuilderBase<PointScalarType>::Vector4 point;

  PointSet<PointType> points;
  points.reserve(userLastAzimutAngleIndex - userFirstAzimutAngleIndex);
  for (size_t n = userFirstAzimutAngleIndex; n <= userLastAzimutAngleIndex; n++) {
    double d = ranges[n];
    if (d > this->minimalRange_ && d < this->maximalRange_) {
      point = this->H_ * (this->rayUnitVectors_[n] * ranges[n] + this->Zero_);
      if (pointRange.inside(point.template segment<PointTraits<PointType>::SIZE>(0))) {
        points.emplace_back(point.template segment<PointTraits<PointType>::SIZE>(0));
      }
    }
  }

  return points;
}

template class LIDAR2DFrameBuilder<Eigen::Vector2f, float>;
template class LIDAR2DFrameBuilder<Eigen::Vector2d, float>;
template class LIDAR2DFrameBuilder<Eigen::Vector3f, float>;
template class LIDAR2DFrameBuilder<Eigen::Vector3d, float>;
template class LIDAR2DFrameBuilder<Eigen::Vector2f, double>;
template class LIDAR2DFrameBuilder<Eigen::Vector2d, double>;
template class LIDAR2DFrameBuilder<Eigen::Vector3f, double>;
template class LIDAR2DFrameBuilder<Eigen::Vector3d, double>;

}  // namespace core
}  // namespace romea
