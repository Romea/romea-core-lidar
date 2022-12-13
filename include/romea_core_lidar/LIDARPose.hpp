#ifndef ROMEA_CORE_LIDAR_LIDARPOSE_HPP_ 
#define ROMEA_CORE_LIDAR_LIDARPOSE_HPP_ 

// std
#include <functional>
#include <memory>

// Eigen
#include "Eigen/Core"

// romea
#include <romea_core_common/time/Time.hpp>

// tbb
#include "tbb/concurrent_priority_queue.h"



namespace romea {


template <typename Scalar>
class LIDARPose
{
public :

  using Matrix4  = Eigen::Matrix<Scalar, 4, 4> ;
  using Vector4  = Eigen::Matrix<Scalar, 4, 1> ;
  using UpdateFunction  = StampedWrapper<Duration, std::function<void(void)> > ;
  using UpdateFunctionPtr = std::shared_ptr<UpdateFunction> ;
  using Self = LIDARPose<Scalar>;

public :

  explicit LIDARPose(const Matrix4 & initialLidarPose);

  void appendAngularVelocities(const Duration & duration,
                               const Scalar & angularSpeedAlongXBodyAxis,
                               const Scalar & angularSpeedAlongYBodyAxis,
                               const Scalar & angularSpeedAlongZBodyAxis);

  void appendLinearVelocities(const Duration & duration,
                              const Scalar & linearSpeedAlongXBodyAxis,
                              const Scalar & linearSpeedAlongYBodyAxis,
                              const Scalar & linearSpeedAlongZBodyAxis);

  void appendTwist(const Duration & duration,
                   const Scalar & angularSpeedAlongXBodyAxis,
                   const Scalar & angularSpeedAlongYBodyAxis,
                   const Scalar & angularSpeedAlongZBodyAxis,
                   const Scalar & linearSpeedAlongXBodyAxis,
                   const Scalar & linearSpeedAlongYBodyAxis,
                   const Scalar & linearSpeedAlongZBodyAxis);

  void extrapolate(const Duration & duration, Matrix4 & extrapolatePose);

  void reset(const Duration & duration);

  void update(const Duration & duration);


private :


  void updateAngularVelocities_(const Scalar & angularSpeedAlongXBodyAxis,
                                const Scalar & angularSpeedAlongYBodyAxis,
                                const Scalar & angularSpeedAlongZBodyAxis);

  void updateLinearVelocities_(const Scalar & linearSpeedAlongXBodyAxis,
                               const Scalar & linearSpeedAlongYBodyAxis,
                               const Scalar & linearSpeedAlongZBodyAxis);

  void updateTwist_(const Scalar & angularSpeedAlongXBodyAxis,
                    const Scalar & angularSpeedAlongYBodyAxis,
                    const Scalar & angularSpeedAlongZBodyAxis,
                    const Scalar & linearSpeedAlongXBodyAxis,
                    const Scalar & linearSpeedAlongYBodyAxis,
                    const Scalar & linearSpeedAlongZBodyAxis);

  void predict_(const Matrix4 & previousPose,
                const Duration &currentStamp,
                Matrix4 & currentPose);

private :

  Duration previousUpdateStamp_;
  UpdateFunction lastUpdateFunction_;

  Matrix4 I_;
  Matrix4 H_;
  Matrix4 twistH_;
  Matrix4 initialH_;
  Matrix4 interpolatedH_;
  tbb::concurrent_priority_queue<UpdateFunction, std::greater<UpdateFunction> > updateFunctionQueue_;

};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR_LIDARPOSE_HPP_ 
