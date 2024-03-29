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


#include "romea_core_lidar/LIDARPose.hpp"
#include <romea_core_common/pointset/PointTraits.hpp>

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
template<typename Scalar>
LIDARPose<Scalar>::LIDARPose(const Matrix4 & initialLidarPose)
: previousUpdateStamp_(Duration::zero()),
  lastUpdateFunction_{Duration::zero(), []() {}},
  I_(Matrix4::Identity()),
  H_(Matrix4::Identity()),
  twistH_(Matrix4::Zero()),
  initialH_(initialLidarPose),
  interpolatedH_(Matrix4::Identity()),
  updateFunctionQueue_(1000)
{
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void
LIDARPose<Scalar>::appendAngularVelocities(
  const Duration & duration,
  const Scalar & angularSpeedAlongXBodyAxis,
  const Scalar & angularSpeedAlongYBodyAxis,
  const Scalar & angularSpeedAlongZBodyAxis)
{
  updateFunctionQueue_.emplace(
    UpdateFunction{duration,
      std::bind(
        &Self::updateAngularVelocities_,
        this,
        angularSpeedAlongXBodyAxis,
        angularSpeedAlongYBodyAxis,
        angularSpeedAlongZBodyAxis)});
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::appendLinearVelocities(
  const Duration & duration,
  const Scalar & linearSpeedAlongXBodyAxis,
  const Scalar & linearSpeedAlongYBodyAxis,
  const Scalar & linearSpeedAlongZBodyAxis)
{
  updateFunctionQueue_.emplace(
    UpdateFunction{duration,
      std::bind(
        &Self::updateLinearVelocities_,
        this,
        linearSpeedAlongXBodyAxis,
        linearSpeedAlongYBodyAxis,
        linearSpeedAlongZBodyAxis)});
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::appendTwist(
  const Duration & duration,
  const Scalar & angularSpeedAlongXBodyAxis,
  const Scalar & angularSpeedAlongYBodyAxis,
  const Scalar & angularSpeedAlongZBodyAxis,
  const Scalar & linearSpeedAlongXBodyAxis,
  const Scalar & linearSpeedAlongYBodyAxis,
  const Scalar & linearSpeedAlongZBodyAxis)
{
  updateFunctionQueue_.emplace(
    UpdateFunction{duration,
      std::bind(
        &Self::updateTwist_,
        this,
        angularSpeedAlongXBodyAxis,
        angularSpeedAlongYBodyAxis,
        angularSpeedAlongZBodyAxis,
        linearSpeedAlongXBodyAxis,
        linearSpeedAlongYBodyAxis,
        linearSpeedAlongZBodyAxis)});
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::updateAngularVelocities_(
  const Scalar & angularSpeedAlongXBodyAxis,
  const Scalar & angularSpeedAlongYBodyAxis,
  const Scalar & angularSpeedAlongZBodyAxis)
{
  twistH_(0, 1) = -angularSpeedAlongZBodyAxis;
  twistH_(0, 2) = angularSpeedAlongYBodyAxis;
  twistH_(1, 0) = angularSpeedAlongZBodyAxis;
  twistH_(1, 2) = -angularSpeedAlongXBodyAxis;
  twistH_(2, 0) = -angularSpeedAlongYBodyAxis;
  twistH_(2, 1) = angularSpeedAlongXBodyAxis;
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::updateLinearVelocities_(
  const Scalar & linearSpeedAlongXBodyAxis,
  const Scalar & linearSpeedAlongYBodyAxis,
  const Scalar & linearSpeedAlongZBodyAxis)
{
  twistH_(0, 3) = linearSpeedAlongXBodyAxis;
  twistH_(1, 3) = linearSpeedAlongYBodyAxis;
  twistH_(2, 3) = linearSpeedAlongZBodyAxis;
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::updateTwist_(
  const Scalar & angularSpeedAlongXBodyAxis,
  const Scalar & angularSpeedAlongYBodyAxis,
  const Scalar & angularSpeedAlongZBodyAxis,
  const Scalar & linearSpeedAlongXBodyAxis,
  const Scalar & linearSpeedAlongYBodyAxis,
  const Scalar & linearSpeedAlongZBodyAxis)
{
  updateAngularVelocities_(
    angularSpeedAlongXBodyAxis,
    angularSpeedAlongYBodyAxis,
    angularSpeedAlongZBodyAxis);

  updateLinearVelocities_(
    linearSpeedAlongXBodyAxis,
    linearSpeedAlongYBodyAxis,
    linearSpeedAlongZBodyAxis);
}


//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::reset(const Duration & duration)
{
  previousUpdateStamp_ = duration;
  this->H_ = initialH_;
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::update(const Duration & duration)
{
  while (lastUpdateFunction_.stamp < duration) {
    if (updateFunctionQueue_.try_pop(lastUpdateFunction_)) {
      predict_(this->H_, lastUpdateFunction_.stamp, this->H_);
      lastUpdateFunction_.data();
      previousUpdateStamp_ = lastUpdateFunction_.stamp;
    } else {
      return;
    }
  }
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::predict_(
  const Matrix4 & previousPose,
  const Duration & currentStamp,
  Matrix4 & currentPose)
{
  auto dt = durationToSecond(currentStamp - previousUpdateStamp_);
  currentPose = (I_ + twistH_ * dt) * previousPose;
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDARPose<Scalar>::extrapolate(const Duration & duration, Matrix4 & extrapolatedPose)
{
  predict_(H_, duration, extrapolatedPose);
}

template class LIDARPose<float>;
template class LIDARPose<double>;

}  // namespace core
}  // namespace romea
