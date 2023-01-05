// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDAR2D_HPP_
