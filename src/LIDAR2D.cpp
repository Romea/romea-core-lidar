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


#include "romea_core_lidar/LIDAR2D.hpp"

namespace romea
{

namespace core
{

//-----------------------------------------------------------------------------
LIDAR2D::LIDAR2D(
  const double & rate,
  const double & minimalAzimutAngle,
  const double & maximalAzimutAngle,
  const double & azimutAngleIncrement,
  const double & azimutAngleStd,
  const double & mininalRange,
  const double & maximalRange,
  const double & rangeStd)
: rate_(rate),
  minimalAzimutAngle_(minimalAzimutAngle),
  maximalAzimutAngle_(maximalAzimutAngle),
  azimutAngleIncrement_(azimutAngleIncrement),
  azimutAngleStd_(azimutAngleStd),
  azimutAperture_(maximalAzimutAngle_ - minimalAzimutAngle_),
  mininalRange_(mininalRange),
  maximalRange_(maximalRange),
  rangeStd_(rangeStd),
  rangeVariance_(rangeStd * rangeStd),
  lidarPose_(Eigen::Affine3d::Identity())
{
}

//-----------------------------------------------------------------------------
void LIDAR2D::setBodyPose(const Eigen::Affine3d & position)
{
  lidarPose_ = position;
}

//-----------------------------------------------------------------------------
const Eigen::Affine3d & LIDAR2D::getBodyPose()const
{
  return lidarPose_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getRate() const
{
  return rate_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getMinimalAzimutAngle() const
{
  return minimalAzimutAngle_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getMaximalAzimutAngle() const
{
  return maximalAzimutAngle_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getAzimutAngleIncrement() const
{
  return azimutAngleIncrement_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getMinimalRange()const
{
  return mininalRange_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getMaximalRange()const
{
  return maximalRange_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getRangeStd()const
{
  return rangeStd_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getRangeVariance()const
{
  return rangeVariance_;
}

//-----------------------------------------------------------------------------
const double & LIDAR2D::getAzimutAperture()const
{
  return azimutAperture_;
}

}  // namespace core
}  // namespace romea
