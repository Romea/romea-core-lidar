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


#ifndef ROMEA_CORE_LIDAR__LIDAR2D_HPP_
#define ROMEA_CORE_LIDAR__LIDAR2D_HPP_

// eigen
#include <Eigen/Geometry>

// romea core
#include <romea_core_common/pointset/PointSet.hpp>

// std
#include <string>
#include <vector>


namespace romea
{
namespace core
{

class LIDAR2D
{
public:
  LIDAR2D(
    const double & rate,
    const double & minimalAzimutAngle,
    const double & maximalAzimutAngle,
    const double & azimutAngleIncrement,
    const double & azimutAngleStd_,
    const double & mininalRange,
    const double & maximalRange,
    const double & rangeStd);

  virtual ~LIDAR2D() = default;

  const double & getRate() const;

  const double & getMinimalAzimutAngle() const;
  const double & getMaximalAzimutAngle() const;
  const double & getAzimutAngleIncrement() const;
  const double & getAzimutAngleStd()const;
  const double & getAzimutAngleVariance()const;
  const double & getAzimutAperture()const;

  const double & getMinimalRange() const;
  const double & getMaximalRange() const;
  const double & getRangeStd()const;
  const double & getRangeVariance()const;

  void setBodyPose(const Eigen::Affine3d & position);
  const Eigen::Affine3d & getBodyPose()const;

protected:
  double rate_;

  double minimalAzimutAngle_;
  double maximalAzimutAngle_;
  double azimutAngleIncrement_;
  double azimutAngleStd_;
  double azimutAngleVariance_;
  double azimutAperture_;

  double mininalRange_;
  double maximalRange_;
  double rangeStd_;
  double rangeVariance_;

  Eigen::Affine3d lidarPose_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDAR2D_HPP_
