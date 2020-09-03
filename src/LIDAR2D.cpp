#include "romea_lidar/LIDAR2D.hpp"

namespace romea {

//-----------------------------------------------------------------------------
LIDAR2D::LIDAR2D(const double & rate,
                 const double & minimalAzimutAngle,
                 const double & maximalAzimutAngle,
                 const double & azimutAngleIncrement,
                 const double & azimutAngleStd,
                 const double & mininalRange,
                 const double & maximalRange,
                 const double & rangeStd):
  rate_(rate),
  minimalAzimutAngle_(minimalAzimutAngle),
  maximalAzimutAngle_(maximalAzimutAngle),
  azimutAngleIncrement_(azimutAngleIncrement),
  azimutAngleStd_(azimutAngleStd),
  azimutAperture_(maximalAzimutAngle_-minimalAzimutAngle_),
  mininalRange_(mininalRange),
  maximalRange_(maximalRange),
  rangeStd_(rangeStd),
  rangeVariance_(rangeStd*rangeStd),
  lidarPose_(Eigen::Affine3d::Identity())
{

}

//-----------------------------------------------------------------------------
void LIDAR2D::setBodyPose(const Eigen::Affine3d & position)
{
  lidarPose_=position;
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

}
