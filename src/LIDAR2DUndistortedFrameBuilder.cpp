#include "romea_core_lidar/LIDAR2DUndistortedFrameBuilder.hpp"
#include <romea_core_common/pointset/PointTraits.hpp>

namespace romea {


//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
LIDAR2DUndistortedFrameBuilder<PointType, RangeScalarType>::
LIDAR2DUndistortedFrameBuilder(const LIDAR2D & lidar):
  LIDAR2DFrameBuilderBase<typename PointType::Scalar>(lidar),
  pose_(lidar.getBodyPose().matrix().cast<Scalar>())

{

}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
void LIDAR2DUndistortedFrameBuilder<PointType, RangeScalarType>::
appendAngularVelocities(const Duration & duration,
                        const Scalar & angularSpeedAlongXBodyAxis,
                        const Scalar & angularSpeedAlongYBodyAxis,
                        const Scalar & angularSpeedAlongZBodyAxis)
{
  pose_.appendAngularVelocities(duration,
                                angularSpeedAlongXBodyAxis,
                                angularSpeedAlongYBodyAxis,
                                angularSpeedAlongZBodyAxis);

}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
void LIDAR2DUndistortedFrameBuilder <PointType, RangeScalarType>::
appendLinearVelocities(const Duration & duration,
                       const Scalar &linearSpeedAlongXBodyAxis,
                       const Scalar &linearSpeedAlongYBodyAxis,
                       const Scalar &linearSpeedAlongZBodyAxis)
{
  pose_.appendLinearVelocities(duration,
                               linearSpeedAlongXBodyAxis,
                               linearSpeedAlongYBodyAxis,
                               linearSpeedAlongZBodyAxis);

}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
void LIDAR2DUndistortedFrameBuilder <PointType, RangeScalarType>::
appendTwist(const Duration & duration,
            const Scalar & angularSpeedAlongXBodyAxis,
            const Scalar & angularSpeedAlongYBodyAxis,
            const Scalar & angularSpeedAlongZBodyAxis,
            const Scalar & linearSpeedAlongXBodyAxis,
            const Scalar & linearSpeedAlongYBodyAxis,
            const Scalar & linearSpeedAlongZBodyAxis)
{
  pose_.appendTwist(duration,
                    angularSpeedAlongXBodyAxis,
                    angularSpeedAlongYBodyAxis,
                    angularSpeedAlongZBodyAxis,
                    linearSpeedAlongXBodyAxis,
                    linearSpeedAlongYBodyAxis,
                    linearSpeedAlongZBodyAxis);
}


//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
LIDARUndistortedFrame<PointType>
LIDAR2DUndistortedFrameBuilder<PointType, RangeScalarType>::
createUndistortedFrame(const Duration &startAcquisitionTime,
                       const Duration &endAcquisitionTime,
                       const RangeVector &ranges)
{
  return createUndistortedFrame(startAcquisitionTime,
                                endAcquisitionTime,
                                ranges,
                                this->minimalRange_,
                                this->maximalRange_,
                                0,
                                this->rayUnitVectors_.size()-1);
}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
LIDARUndistortedFrame<PointType>
LIDAR2DUndistortedFrameBuilder<PointType, RangeScalarType>::
createUndistortedFrame(const Duration &startAcquisitionTime,
                       const Duration &endAcquisitionTime,
                       const RangeVector & ranges,
                       const double & userMinimalRange,
                       const double & userMaximalRange)
{
  return createUndistortedFrame(startAcquisitionTime,
                                endAcquisitionTime,
                                ranges,
                                userMinimalRange,
                                userMaximalRange,
                                0,
                                this->rayUnitVectors_.size()-1);
}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
LIDARUndistortedFrame<PointType>
LIDAR2DUndistortedFrameBuilder<PointType, RangeScalarType>::
createUndistortedFrame(const Duration &startAcquisitionTime,
                       const Duration &endAcquisitionTime,
                       const RangeVector & ranges,
                       const double & userMinimalRange,
                       const double & userMaximalRange,
                       const size_t & userFirstAzimutAngleIndex,
                       const size_t & userLastAzimutAngleIndex)
{
  assert(userFirstAzimutAngleIndex < this->rayUnitVectors_.size());
  assert(userLastAzimutAngleIndex < this->rayUnitVectors_.size());

  Duration t = startAcquisitionTime;
  Duration dt = (endAcquisitionTime-startAcquisitionTime)/
      (userLastAzimutAngleIndex-userFirstAzimutAngleIndex);

  pose_.reset(t);
  pose_.update(t);

  LIDARUndistortedFrame<PointType> frame;
  frame.laserOriginPoints.reserve(this->rayUnitVectors_.size());
  frame.laserEndPoints.reserve(this->rayUnitVectors_.size());
  frame.startAcquisitionTime = startAcquisitionTime;
  frame.endAcquisitionTime = endAcquisitionTime;


  Vector4 point;
  double minimalRange = std::max(this->minimalRange_, userMinimalRange);
  double maximalRange = std::min(this->maximalRange_, userMaximalRange);

  for (size_t n=userFirstAzimutAngleIndex; n <= userLastAzimutAngleIndex; n++, t+=dt)
  {
    if (ranges[n] > minimalRange && ranges[n] < maximalRange)
    {
      pose_.extrapolate(t, this->H_);
      point = this->H_*this->Zero_;
      frame.laserOriginPoints.push_back(point.template segment<PointTraits<PointType>::SIZE>(0));
      point += this->H_*(this->rayUnitVectors_[n]*ranges[n]+this->Zero_);
      frame.laserEndPoints.push_back(point.template segment<PointTraits<PointType>::SIZE>(0));
    }
  }
  return frame;
}

template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector2f, float>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector2d, float>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector3f, float>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector3d, float>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector2f, double>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector2d, double>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector3f, double>;
template class LIDAR2DUndistortedFrameBuilder<Eigen::Vector3d, double>;

}  // namespace romea

