#ifndef romea_LIDARUndistortedFrame_hpp
#define romea_LIDARUndistortedFrame_hpp

//romea
#include "romea_core_common/pointset/PointSet.hpp"
#include "romea_core_common/time/Time.hpp"

//std
#include <memory>

namespace romea {

template <class PointType>
struct LIDARUndistortedFrame
{
  using Ptr = std::shared_ptr<LIDARUndistortedFrame<PointType> > ;
  using ConstPtr = std::shared_ptr<const LIDARUndistortedFrame<PointType> > ;

  romea::Duration startAcquisitionTime;
  romea::Duration endAcquisitionTime;
  Eigen::Matrix<typename PointType::Scalar,4,4> startLidarPose;
  Eigen::Matrix<typename PointType::Scalar,4,4> endLidarPose;
  PointSet<PointType> laserOriginPoints;
  PointSet<PointType> laserEndPoints;
};

}

#endif
