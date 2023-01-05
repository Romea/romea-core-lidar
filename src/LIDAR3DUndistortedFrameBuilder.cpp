// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea core
#include <romea_core_common/pointset/PointTraits.hpp>

// std
#include <algorithm>

// local
#include "romea_core_lidar/LIDAR3DUndistortedFrameBuilder.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::LIDAR3DUndistortedFrameBuilder(
  const LIDAR3D & lidar)
: LIDAR3DFrameBuilderBase<typename PointType::Scalar>(lidar),
  pose_(lidar.getBodyPose().matrix().cast<Scalar>())
{
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
void LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::appendAngularVelocities(
  const Duration & duration,
  const Scalar & angularSpeedAlongXBodyAxis,
  const Scalar & angularSpeedAlongYBodyAxis,
  const Scalar & angularSpeedAlongZBodyAxis)
{
  pose_.appendAngularVelocities(
    duration,
    angularSpeedAlongXBodyAxis,
    angularSpeedAlongYBodyAxis,
    angularSpeedAlongZBodyAxis);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
void LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::appendLinearVelocities(
  const Duration & duration,
  const Scalar & linearSpeedAlongXBodyAxis,
  const Scalar & linearSpeedAlongYBodyAxis,
  const Scalar & linearSpeedAlongZBodyAxis)
{
  pose_.appendLinearVelocities(
    duration,
    linearSpeedAlongXBodyAxis,
    linearSpeedAlongYBodyAxis,
    linearSpeedAlongZBodyAxis);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
void LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::appendTwist(
  const Duration & duration,
  const Scalar & angularSpeedAlongXBodyAxis,
  const Scalar & angularSpeedAlongYBodyAxis,
  const Scalar & angularSpeedAlongZBodyAxis,
  const Scalar & linearSpeedAlongXBodyAxis,
  const Scalar & linearSpeedAlongYBodyAxis,
  const Scalar & linearSpeedAlongZBodyAxis)
{
  pose_.appendTwist(
    duration,
    angularSpeedAlongXBodyAxis,
    angularSpeedAlongYBodyAxis,
    angularSpeedAlongZBodyAxis,
    linearSpeedAlongXBodyAxis,
    linearSpeedAlongYBodyAxis,
    linearSpeedAlongZBodyAxis);
}


//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
LIDARUndistortedFrame<PointType>
LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::createUndistortedFrame(
  const Duration & startAcquisitionTime,
  const Duration & endAcquisitionTime,
  const RangeVector & ranges)
{
  return createUndistortedFrame(
    startAcquisitionTime,
    endAcquisitionTime,
    ranges,
    this->minimalRange_,
    this->maximalRange_,
    0,
    this->numberOfAzimutAngles_ - 1,
    0.,
    this->numberOfElevationAngles_ - 1);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
LIDARUndistortedFrame<PointType>
LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::createUndistortedFrame(
  const Duration & startAcquisitionTime,
  const Duration & endAcquisitionTime,
  const RangeVector & ranges,
  const double & userMinimalRange,
  const double & userMaximalRange)
{
  return createUndistortedFrame(
    startAcquisitionTime,
    endAcquisitionTime,
    ranges,
    userMinimalRange,
    userMaximalRange,
    0,
    this->numberOfAzimutAngles_ - 1,
    0.,
    this->numberOfElevationAngles_ - 1);
}

//-----------------------------------------------------------------------------
template<class PointType, typename RangeScalarType>
LIDARUndistortedFrame<PointType>
LIDAR3DUndistortedFrameBuilder<PointType, RangeScalarType>::createUndistortedFrame(
  const Duration & startAcquisitionTime,
  const Duration & endAcquisitionTime,
  const RangeVector & ranges,
  const double & userMinimalRange,
  const double & userMaximalRange,
  const size_t & userFirstAzimutAngleIndex,
  const size_t & userLastAzimutAngleIndex,
  const size_t & userFirstElevationAngleIndex,
  const size_t & userLastElevationAngleIndex)
{
  assert(userFirstAzimutAngleIndex < this->numberOfAzimutAngles_);
  assert(userLastAzimutAngleIndex < this->numberOfAzimutAngles_);
  assert(userFirstElevationAngleIndex < this->numberOfElevationAngles_);
  assert(userLastElevationAngleIndex < this->numberOfElevationAngles_);

  size_t numberOfRanges = (userLastAzimutAngleIndex - userFirstAzimutAngleIndex) *
    (userLastElevationAngleIndex - userFirstElevationAngleIndex);

  Duration t = startAcquisitionTime;
  Duration dt = endAcquisitionTime - startAcquisitionTime / numberOfRanges;
  pose_.reset(t);
  pose_.update(t);

  LIDARUndistortedFrame<PointType> frame;
  frame.laserOriginPoints.reserve(numberOfRanges);
  frame.laserEndPoints.reserve(numberOfRanges);
  frame.startAcquisitionTime = startAcquisitionTime;
  frame.endAcquisitionTime = endAcquisitionTime;


  Vector4 point;
  double minimalRange = std::max(this->minimalRange_, userMinimalRange);
  double maximalRange = std::min(this->maximalRange_, userMaximalRange);

  size_t rayIndex = 0;
  if (this->scanStorageOrder_ == LIDAR3D::ScanStorageOrder::ELEVATION_MAJOR) {
    for (size_t i = userFirstAzimutAngleIndex; i <= userLastAzimutAngleIndex; i++) {
      for (size_t j = userFirstElevationAngleIndex; j <= userLastElevationAngleIndex;
        j++, rayIndex++, t += dt)
      {
        pose_.update(t);
        size_t rangeIndex = i * this->numberOfElevationAngles_ + j;
        if (ranges[rangeIndex] > minimalRange && ranges[rangeIndex] < maximalRange) {
          pose_.extrapolate(t, this->H_);

          point = this->H_ * this->Zero_;
          frame.laserOriginPoints.emplace_back(
            point.template segment<PointTraits<PointType>::SIZE>(0));

          point += this->H_ * (this->rayUnitVectors_[rayIndex] * ranges[rangeIndex] + this->Zero_);
          frame.laserEndPoints.emplace_back(
            point.template segment<PointTraits<PointType>::SIZE>(0));
        }
      }
    }
  } else {
    for (size_t j = userFirstElevationAngleIndex; j <= userLastElevationAngleIndex;
      j++, rayIndex++)
    {
      for (size_t i = userFirstAzimutAngleIndex; i <= userLastAzimutAngleIndex; i++, t += dt) {
        pose_.update(t);
        size_t rangeIndex = j * this->numberOfAzimutAngles_ + i;
        if (ranges[rangeIndex] > minimalRange && ranges[rangeIndex] < maximalRange) {
          pose_.extrapolate(t, this->H_);

          point = this->H_ * this->Zero_;
          frame.laserOriginPoints.emplace_back(
            point.template segment<PointTraits<PointType>::SIZE>(0));

          point += this->H_ * this->rayUnitVectors_[rayIndex] * ranges[rangeIndex];
          frame.laserEndPoints.emplace_back(
            point.template segment<PointTraits<PointType>::SIZE>(0));
        }
      }
    }
  }
  return frame;
}

template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector2f, float>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector2d, float>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector3f, float>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector3d, float>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector2f, double>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector2d, double>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector3f, double>;
template class LIDAR3DUndistortedFrameBuilder<Eigen::Vector3d, double>;

}  // namespace romea
