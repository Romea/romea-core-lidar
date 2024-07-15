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


#ifndef ROMEA_CORE_LIDAR__LIDARPOSE_HPP_
#define ROMEA_CORE_LIDAR__LIDARPOSE_HPP_

// std
#include <functional>
#include <memory>

// Eigen
#include "Eigen/Core"

// tbb
#include "tbb/concurrent_priority_queue.h"

// romea
#include "romea_core_common/time/Time.hpp"


namespace romea
{
namespace core
{


template<typename Scalar>
class LIDARPose
{
public:
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
  using UpdateFunction = StampedWrapper<Duration, std::function<void (void)>>;
  using UpdateFunctionPtr = std::shared_ptr<UpdateFunction>;
  using Self = LIDARPose<Scalar>;

public:
  explicit LIDARPose(const Matrix4 & initialLidarPose);

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

  void extrapolate(const Duration & duration, Matrix4 & extrapolatePose);

  void reset(const Duration & duration);

  void update(const Duration & duration);

private:
  void updateAngularVelocities_(
    const Scalar & angularSpeedAlongXBodyAxis,
    const Scalar & angularSpeedAlongYBodyAxis,
    const Scalar & angularSpeedAlongZBodyAxis);

  void updateLinearVelocities_(
    const Scalar & linearSpeedAlongXBodyAxis,
    const Scalar & linearSpeedAlongYBodyAxis,
    const Scalar & linearSpeedAlongZBodyAxis);

  void updateTwist_(
    const Scalar & angularSpeedAlongXBodyAxis,
    const Scalar & angularSpeedAlongYBodyAxis,
    const Scalar & angularSpeedAlongZBodyAxis,
    const Scalar & linearSpeedAlongXBodyAxis,
    const Scalar & linearSpeedAlongYBodyAxis,
    const Scalar & linearSpeedAlongZBodyAxis);

  void predict_(
    const Matrix4 & previousPose,
    const Duration & currentStamp,
    Matrix4 & currentPose);

private:
  Duration previousUpdateStamp_;
  UpdateFunction lastUpdateFunction_;

  Matrix4 I_;
  Matrix4 H_;
  Matrix4 twistH_;
  Matrix4 initialH_;
  Matrix4 interpolatedH_;
  tbb::concurrent_priority_queue<UpdateFunction, std::greater<UpdateFunction>> updateFunctionQueue_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDARPOSE_HPP_
