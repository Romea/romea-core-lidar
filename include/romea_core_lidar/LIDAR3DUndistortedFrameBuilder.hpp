#ifndef romea_LIDAR3DUndistortedFrameBuilder_hpp
#define romea_LIDAR3DUndistortedFrameBuilder_hpp

//romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/Interval.hpp>
#include <romea_core_common/pointset/PointTraits.hpp>

#include "LIDAR3DFrameBuilderBase.hpp"
#include "LIDARUndistortedFrame.hpp"
#include "LIDARPose.hpp"

//tbb
#include "tbb/concurrent_priority_queue.h"

//std
#include <functional>

namespace romea {


template <class PointType, typename RangeScalarType>
class LIDAR3DUndistortedFrameBuilder : public LIDAR3DFrameBuilderBase<typename PointType::Scalar>
{  

public :

  using Scalar =typename PointType::Scalar ;
  using Matrix4 =Eigen::Matrix<Scalar,4,4> ;
  using Vector4 =Eigen::Matrix<Scalar,4,1> ;
  using UpdateFunction = StampedWrapper<Duration,std::function<void(void)> > ;
  using UpdateFunctionPtr = std::shared_ptr<UpdateFunction> ;
  using Self = LIDAR3DUndistortedFrameBuilder<PointType,RangeScalarType>;

  using RangeVector = std::vector<RangeScalarType> ;
  using PointRange = IntervalComplement<Scalar,PointTraits<PointType>::DIM>;

public :

  LIDAR3DUndistortedFrameBuilder(const LIDAR3D & lidar);


  virtual ~LIDAR3DUndistortedFrameBuilder()=default;


public :

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

  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
                                                          const Duration &endAcquisitionTime,
                                                          const RangeVector & ranges);

  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
                                                          const Duration &endAcquisitionTime,
                                                          const RangeVector & ranges,
                                                          const double & userMinimalRange,
                                                          const double & userMaximalRange);

  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
                                                          const Duration &endAcquisitionTime,
                                                          const RangeVector & ranges,
                                                          const double & userMinimalRange,
                                                          const double & userMaximalRange,
                                                          const size_t & userFirstAzimutAngleIndex,
                                                          const size_t & userLastAzimutAngleIndex,
                                                          const size_t & userFirstElevationAngleIndex,
                                                          const size_t & userLastElevationAngleIndex);

  //  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
  //                                                          const Duration &endAcquisitionTime,
  //                                                          const RangeVector & ranges,
  //                                                          const RangeSet & rangeSet,
  //                                                          const size_t & userFirstAzimutAngleIndex,
  //                                                          const size_t & userLastAzimutAngleIndex,
  //                                                          const size_t & userFirstElevationAngleIndex,
  //                                                          const size_t & userLastElevationAngleIndex);

  //  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
  //                                                          const Duration &endAcquisitionTime,
  //                                                          const RangeVector & ranges,
  //                                                          const RangeSet & rangeSet,
  //                                                          const size_t & firstAzimutAngleIndex,
  //                                                          const size_t & lastAzimutAngleIndex);
  //                                                          const size_t & userFirstElevationAngleIndex,
  //                                                          const size_t & userLastElevationAngleIndex);
protected :

  LIDARPose<Scalar> pose_;
};




}

#endif
