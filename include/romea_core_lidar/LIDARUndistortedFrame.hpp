// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LIDAR__LIDARUNDISTORTEDFRAME_HPP
#define ROMEA_CORE_LIDAR__LIDARUNDISTORTEDFRAME_HPP

// std
#include <memory>

// romea
#include "romea_core_common/pointset/PointSet.hpp"
#include "romea_core_common/time/Time.hpp"


namespace romea
{

template<class PointType>
struct LIDARUndistortedFrame
{
  using Ptr = std::shared_ptr<LIDARUndistortedFrame<PointType>>;
  using ConstPtr = std::shared_ptr<const LIDARUndistortedFrame<PointType>>;

  romea::Duration startAcquisitionTime;
  romea::Duration endAcquisitionTime;
  Eigen::Matrix<typename PointType::Scalar, 4, 4> startLidarPose;
  Eigen::Matrix<typename PointType::Scalar, 4, 4> endLidarPose;
  PointSet<PointType> laserOriginPoints;
  PointSet<PointType> laserEndPoints;
};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDARUNDISTORTEDFRAME_HPP
