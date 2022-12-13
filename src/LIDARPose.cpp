#include "romea_core_lidar/LIDARPose.hpp"
#include <romea_core_common/pointset/PointTraits.hpp>

namespace romea {


//-----------------------------------------------------------------------------
template <typename Scalar>
LIDARPose<Scalar>::LIDARPose(const Matrix4 &initialLidarPose):
  previousUpdateStamp_(Duration::zero()),
  lastUpdateFunction_{Duration::zero(), [](){}},
  I_(Matrix4::Identity()),
  H_(Matrix4::Identity()),
  twistH_(Matrix4::Zero()),
  initialH_(initialLidarPose),
  interpolatedH_(Matrix4::Identity()),
  updateFunctionQueue_(1000)
{

}

//-----------------------------------------------------------------------------
template <typename Scalar> void
LIDARPose <Scalar>::appendAngularVelocities(const Duration & duration,
                                            const Scalar & angularSpeedAlongXBodyAxis,
                                            const Scalar & angularSpeedAlongYBodyAxis,
                                            const Scalar & angularSpeedAlongZBodyAxis)
{
  updateFunctionQueue_.emplace(UpdateFunction{duration,
                                              std::bind(&Self::updateAngularVelocities_,
                                              this,
                                              angularSpeedAlongXBodyAxis,
                                              angularSpeedAlongYBodyAxis,
                                              angularSpeedAlongZBodyAxis)});
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void  LIDARPose <Scalar>::appendLinearVelocities(const Duration & duration,
                                                 const Scalar &linearSpeedAlongXBodyAxis,
                                                 const Scalar &linearSpeedAlongYBodyAxis,
                                                 const Scalar &linearSpeedAlongZBodyAxis)
{
  updateFunctionQueue_.emplace(UpdateFunction{duration,
                                              std::bind(&Self::updateLinearVelocities_,
                                              this,
                                              linearSpeedAlongXBodyAxis,
                                              linearSpeedAlongYBodyAxis,
                                              linearSpeedAlongZBodyAxis)});
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::
appendTwist(const Duration & duration,
            const Scalar & angularSpeedAlongXBodyAxis,
            const Scalar & angularSpeedAlongYBodyAxis,
            const Scalar & angularSpeedAlongZBodyAxis,
            const Scalar & linearSpeedAlongXBodyAxis,
            const Scalar & linearSpeedAlongYBodyAxis,
            const Scalar & linearSpeedAlongZBodyAxis)
{
  updateFunctionQueue_.emplace(UpdateFunction{duration,
                                              std::bind(&Self::updateTwist_,
                                              this,
                                              angularSpeedAlongXBodyAxis,
                                              angularSpeedAlongYBodyAxis,
                                              angularSpeedAlongZBodyAxis,
                                              linearSpeedAlongXBodyAxis,
                                              linearSpeedAlongYBodyAxis,
                                              linearSpeedAlongZBodyAxis)});
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::updateAngularVelocities_(const Scalar & angularSpeedAlongXBodyAxis,
                                                  const Scalar & angularSpeedAlongYBodyAxis,
                                                  const Scalar & angularSpeedAlongZBodyAxis)
{
  twistH_(0, 1) = -angularSpeedAlongZBodyAxis;
  twistH_(0, 2) =  angularSpeedAlongYBodyAxis;
  twistH_(1, 0) =  angularSpeedAlongZBodyAxis;
  twistH_(1, 2) = -angularSpeedAlongXBodyAxis;
  twistH_(2, 0) = -angularSpeedAlongYBodyAxis;
  twistH_(2, 1) =  angularSpeedAlongXBodyAxis;
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::updateLinearVelocities_(const Scalar & linearSpeedAlongXBodyAxis,
                                                 const Scalar & linearSpeedAlongYBodyAxis,
                                                 const Scalar & linearSpeedAlongZBodyAxis)
{
  twistH_(0, 3) =  linearSpeedAlongXBodyAxis;
  twistH_(1, 3) =  linearSpeedAlongYBodyAxis;
  twistH_(2, 3) =  linearSpeedAlongZBodyAxis;
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::updateTwist_(const Scalar & angularSpeedAlongXBodyAxis,
                                      const Scalar & angularSpeedAlongYBodyAxis,
                                      const Scalar & angularSpeedAlongZBodyAxis,
                                      const Scalar & linearSpeedAlongXBodyAxis,
                                      const Scalar & linearSpeedAlongYBodyAxis,
                                      const Scalar & linearSpeedAlongZBodyAxis)
{
  updateAngularVelocities_(angularSpeedAlongXBodyAxis,
                           angularSpeedAlongYBodyAxis,
                           angularSpeedAlongZBodyAxis);

  updateLinearVelocities_(linearSpeedAlongXBodyAxis,
                          linearSpeedAlongYBodyAxis,
                          linearSpeedAlongZBodyAxis);
}


//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::reset(const Duration & duration)
{
  previousUpdateStamp_ = duration;
  this->H_ = initialH_;
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::update(const Duration &duration)
{
  while (lastUpdateFunction_.stamp < duration)
  {
    if (updateFunctionQueue_.try_pop(lastUpdateFunction_))
    {
      predict_(this->H_, lastUpdateFunction_.stamp, this->H_);
      lastUpdateFunction_.data();
      previousUpdateStamp_ = lastUpdateFunction_.stamp;
    } else {
      return;
    }
  }
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose <Scalar>::predict_(const Matrix4 & previousPose,
                                  const Duration & currentStamp,
                                  Matrix4 & currentPose)
{
  auto dt = durationToSecond(currentStamp - previousUpdateStamp_);
  currentPose = (I_ + twistH_*dt)*previousPose;
}

//-----------------------------------------------------------------------------
template <typename Scalar>
void LIDARPose<Scalar>::extrapolate(const Duration & duration, Matrix4 & extrapolatedPose)
{
  predict_(H_, duration, extrapolatedPose);
}

template class LIDARPose<float>;
template class LIDARPose<double>;

}  // namespace romea

