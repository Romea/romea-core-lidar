#ifndef ROMEA_CORE_LIDAR_LIDAR2DFRAMEBUILDER_HPP_ 
#define ROMEA_CORE_LIDAR_LIDAR2DFRAMEBUILDER_HPP_ 

// std
#include <vector>

// romea
#include <romea_core_common/math/Interval.hpp>
#include <romea_core_common/pointset/PointTraits.hpp>
#include "romea_core_lidar/LIDAR2DFrameBuilderBase.hpp"

namespace romea {


template <class PointType, typename RangeScalarType  = float>
class LIDAR2DFrameBuilder : public LIDAR2DFrameBuilderBase<typename PointType::Scalar>
{
public :

  using RangeVector = std::vector<RangeScalarType>;

  using PointScalarType = typename PointType::Scalar;
  using PointRange = IntervalComplement<PointScalarType, PointTraits<PointType>::DIM>;

public :

  LIDAR2DFrameBuilder();

  explicit LIDAR2DFrameBuilder(const LIDAR2D & lidar);

  virtual ~LIDAR2DFrameBuilder() = default;

  PointSet<PointType> createFrame(const RangeVector & ranges)const;

  PointSet<PointType> createFrame(const RangeVector & ranges,
                                  const double & userMinimalRange,
                                  const double & userMaximalRange)const;

  PointSet<PointType> createFrame(const RangeVector & ranges,
                                  const double & userMinimalRange,
                                  const double & userMaximalRange,
                                  const size_t & firstAzimutAngleIndex,
                                  const size_t & lastAzimutAngleIndex)const;

  PointSet<PointType> createFrame(const RangeVector & ranges,
                                  const PointRange & pointRange)const;

  PointSet<PointType> createFrame(const RangeVector & ranges,
                                  const PointRange & pointRange,
                                  const size_t & firstAzimutAngleIndex,
                                  const size_t & lastAzimutAngleIndex)const;
};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR_LIDAR2DFRAMEBUILDER_HPP_
