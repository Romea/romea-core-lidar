#ifndef romea_LIDAR3DFrameBuilder_hpp
#define romea_LIDAR3DFrameBuilder_hpp

//romea
#include <romea_core_common/math/Interval.hpp>
#include <romea_core_common/pointset/PointTraits.hpp>
#include "LIDAR3DFrameBuilderBase.hpp"

namespace romea {


template <class PointType, typename RangeScalarType>
class LIDAR3DFrameBuilder : public LIDAR3DFrameBuilderBase<typename PointType::Scalar>
{

public :

  using RangeVector = std::vector<RangeScalarType>;

  using PointScalarType = typename PointType::Scalar;
  using PointRange = IntervalComplement<PointScalarType,PointTraits<PointType>::DIM>;

public :

  LIDAR3DFrameBuilder();

  LIDAR3DFrameBuilder(const LIDAR3D & lidar);

  virtual ~LIDAR3DFrameBuilder()=default;

  PointSet<PointType> createFrame(const RangeVector & ranges)const;

  PointSet<PointType> createFrame(const RangeVector & ranges,
                                    const double & userMinimalRange,
                                    const double & userMaximalRange)const;

  PointSet<PointType> createFrame(const RangeVector & ranges,
                                    const double & userMinimalRange,
                                    const double & userMaximalRange,
                                    const size_t & userFirstAzimutAngleIndex,
                                    const size_t & userLastAzimutAngleIndex,
                                    const size_t & userFirstElevationAngleIndex,
                                    const size_t & userLastElevationAngleIndex)const;

  //  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
  //                                                          const Duration &endAcquisitionTime,
  //                                                          const RangeVector & ranges,
  //                                                          const PointRange & pointRange,
  //                                                          const size_t & userFirstAzimutAngleIndex,
  //                                                          const size_t & userLastAzimutAngleIndex,
  //                                                          const size_t & userFirstElevationAngleIndex,
  //                                                          const size_t & userLastElevationAngleIndex);

  //  LIDARUndistortedFrame<PointType> createUndistortedFrame(const Duration &startAcquisitionTime,
  //                                                          const Duration &endAcquisitionTime,
  //                                                          const RangeVector & ranges,
  //                                                          const PointRange & pointRange,
  //                                                          const size_t & firstAzimutAngleIndex,
  //                                                          const size_t & lastAzimutAngleIndex);
  //                                                          const size_t & userFirstElevationAngleIndex,
  //                                                          const size_t & userLastElevationAngleIndex);
};

}
#endif
