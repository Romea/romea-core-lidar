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


#ifndef ROMEA_CORE_LIDAR__LIDAR2DUNDISTORTEDFRAMEBUILDER_HPP_
#define ROMEA_CORE_LIDAR__LIDAR2DUNDISTORTEDFRAMEBUILDER_HPP_

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/Interval.hpp>
#include <romea_core_common/pointset/PointTraits.hpp>

// tbb
#include <tbb/concurrent_priority_queue.h>

// std
#include <functional>
#include <memory>
#include <vector>

// local
#include "romea_core_lidar/LIDAR2DFrameBuilderBase.hpp"
#include "romea_core_lidar/LIDARUndistortedFrame.hpp"
#include "romea_core_lidar/LIDARPose.hpp"


namespace romea
{


template<class PointType, typename RangeScalarType = float>
class LIDAR2DUndistortedFrameBuilder : public LIDAR2DFrameBuilderBase<typename PointType::Scalar>
{
public:
  using Scalar = typename PointType::Scalar;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
  using UpdateFunction = StampedWrapper<Duration, std::function<void (void)>>;
  using UpdateFunctionPtr = std::shared_ptr<UpdateFunction>;
  using Self = LIDAR2DUndistortedFrameBuilder<PointType, RangeScalarType>;

  using RangeVector = std::vector<RangeScalarType>;
  using PointRange = IntervalComplement<Scalar, PointTraits<PointType>::DIM>;

public:
  explicit LIDAR2DUndistortedFrameBuilder(const LIDAR2D & lidar);

  virtual ~LIDAR2DUndistortedFrameBuilder() = default;

public:
  void appendAngularVelocities(
    const Duration & duration,
    const Scalar & angularSpeedAlongXBodyAxis,
    const Scalar & angularSpeedAlongYBodyAxis,
    const Scalar & angularSpeedAlongZBodyAxis);

  void appendLinearVelocities(
    const Duration & duration,
    const Scalar & linearSpeedAlongXBodyAxis,
    const Scalar & linearSpeedAlongYBodyAxis,
    const Scalar & linearSpeedAlongZBodyAxis);

  void appendTwist(
    const Duration & duration,
    const Scalar & angularSpeedAlongXBodyAxis,
    const Scalar & angularSpeedAlongYBodyAxis,
    const Scalar & angularSpeedAlongZBodyAxis,
    const Scalar & linearSpeedAlongXBodyAxis,
    const Scalar & linearSpeedAlongYBodyAxis,
    const Scalar & linearSpeedAlongZBodyAxis);

  LIDARUndistortedFrame<PointType> createUndistortedFrame(
    const Duration & startAcquisitionTime,
    const Duration & endAcquisitionTime,
    const RangeVector & ranges);

  LIDARUndistortedFrame<PointType> createUndistortedFrame(
    const Duration & startAcquisitionTime,
    const Duration & endAcquisitionTime,
    const RangeVector & ranges,
    const double & userMinimalRange,
    const double & userMaximalRange);

  LIDARUndistortedFrame<PointType> createUndistortedFrame(
    const Duration & startAcquisitionTime,
    const Duration & endAcquisitionTime,
    const RangeVector & ranges,
    const double & userMinimalRange,
    const double & userMaximalRange,
    const size_t & firstAzimutAngleIndex,
    const size_t & lastAzimutAngleIndex);

//  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
//                                                          const Duration &endAcquisitionTime,
//                                                          const RangeVector & ranges,
//                                                          const PointRange & PointRange);

//  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
//                                                          const Duration &endAcquisitionTime,
//                                                          const RangeVector & ranges,
//                                                          const PointRange & PointRange,
//                                                          const size_t & firstAzimutAngleIndex,
//                                                          const size_t & lastAzimutAngleIndex);

protected:
  LIDARPose<Scalar> pose_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDAR2DUNDISTORTEDFRAMEBUILDER_HPP_
