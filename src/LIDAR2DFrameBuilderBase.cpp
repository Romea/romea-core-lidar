// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea core
#include <romea_core_common/pointset/PointTraits.hpp>

// std
#include <limits>

// local
#include "romea_core_lidar/LIDAR2DFrameBuilderBase.hpp"

namespace romea
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

}  // namespace romea
