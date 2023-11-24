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


// romea core
#include <romea_core_common/pointset/PointTraits.hpp>

// std
#include <limits>

// local
#include "romea_core_lidar/LIDAR2DFrameBuilderBase.hpp"

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
template<class Scalar>
LIDAR2DFrameBuilderBase<Scalar>::LIDAR2DFrameBuilderBase()
: H_(),
  Zero_(Vector4(0, 0, 0, 1)),
  minimalRange_(0),
  maximalRange_(std::numeric_limits<Scalar>::max()),
  rayUnitVectors_()
{
}

//-----------------------------------------------------------------------------
template<class Scalar>
LIDAR2DFrameBuilderBase<Scalar>::LIDAR2DFrameBuilderBase(const LIDAR2D & lidar)
{
  init(lidar);
}


//-----------------------------------------------------------------------------
template<class Scalar>
void LIDAR2DFrameBuilderBase<Scalar>::init(const LIDAR2D & lidar)
{
  H_ = lidar.getBodyPose().matrix().cast<Scalar>();
  minimalRange_ = lidar.getMinimalRange();
  maximalRange_ = lidar.getMaximalRange();

  size_t numberOfRays = size_t(lidar.getAzimutAperture() / lidar.getAzimutAngleIncrement()) + 1;

  rayUnitVectors_.resize(numberOfRays, Vector4(0, 0, 0, 0));
  for (size_t n = 0; n < numberOfRays; ++n) {
    double azimutAngle = lidar.getMinimalAzimutAngle() + n * lidar.getAzimutAngleIncrement();
    rayUnitVectors_[n](0) = std::cos(azimutAngle);
    rayUnitVectors_[n](1) = std::sin(azimutAngle);
  }
}

template class LIDAR2DFrameBuilderBase<float>;
template class LIDAR2DFrameBuilderBase<double>;

}  // namespace core
}  // namespace romea
