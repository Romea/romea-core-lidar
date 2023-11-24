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


#ifndef ROMEA_CORE_LIDAR__LIDAR3DFRAMEBUILDER_HPP
#define ROMEA_CORE_LIDAR__LIDAR3DFRAMEBUILDER_HPP

// romea core
#include <romea_core_common/math/Interval.hpp>
#include <romea_core_common/pointset/PointTraits.hpp>

// std
#include <vector>

// local
#include "romea_core_lidar/LIDAR3DFrameBuilderBase.hpp"

namespace romea
{
namespace core
{


template<class PointType, typename RangeScalarType>
class LIDAR3DFrameBuilder : public LIDAR3DFrameBuilderBase<typename PointType::Scalar>
{
public:
  using RangeVector = std::vector<RangeScalarType>;

  using PointScalarType = typename PointType::Scalar;
  using PointRange = IntervalComplement<PointScalarType, PointTraits<PointType>::DIM>;

public:
  LIDAR3DFrameBuilder();

  explicit LIDAR3DFrameBuilder(const LIDAR3D & lidar);

  virtual ~LIDAR3DFrameBuilder() = default;

  PointSet<PointType> createFrame(const RangeVector & ranges)const;

  PointSet<PointType> createFrame(
    const RangeVector & ranges,
    const double & userMinimalRange,
    const double & userMaximalRange)const;

  PointSet<PointType> createFrame(
    const RangeVector & ranges,
    const double & userMinimalRange,
    const double & userMaximalRange,
    const size_t & userFirstAzimutAngleIndex,
    const size_t & userLastAzimutAngleIndex,
    const size_t & userFirstElevationAngleIndex,
    const size_t & userLastElevationAngleIndex)const;

  //  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
  //                                                          const Duration &endAcquisitionTime,
  //                                                          const RangeVector & ranges,
  //                                                          const PointRange & pointRange,
  //                                                          const size_t & userFirstAzimutAngleIndex,
  //                                                          const size_t & userLastAzimutAngleIndex,
  //                                                          const size_t & userFirstElevationAngleIndex,
  //                                                          const size_t & userLastElevationAngleIndex);

  //  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
  //                                                          const Duration &endAcquisitionTime,
  //                                                          const RangeVector & ranges,
  //                                                          const PointRange & pointRange,
  //                                                          const size_t & firstAzimutAngleIndex,
  //                                                          const size_t & lastAzimutAngleIndex);
  //                                                          const size_t & userFirstElevationAngleIndex,
  //                                                          const size_t & userLastElevationAngleIndex);
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDAR3DFRAMEBUILDER_HPP
