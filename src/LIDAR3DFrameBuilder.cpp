#include "romea_core_lidar/LIDAR3DFrameBuilder.hpp"
#include <romea_core_common/pointset/PointTraits.hpp>

namespace romea {


//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
LIDAR3DFrameBuilder<PointType, RangeScalarType>::LIDAR3DFrameBuilder():
  LIDAR3DFrameBuilderBase<typename PointType::Scalar>()
{
}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType>
LIDAR3DFrameBuilder<PointType, RangeScalarType>::LIDAR3DFrameBuilder(const LIDAR3D &  lidar):
  LIDAR3DFrameBuilderBase<typename PointType::Scalar>(lidar)
{
}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType> PointSet<PointType>
LIDAR3DFrameBuilder<PointType, RangeScalarType>::createFrame(const RangeVector & ranges)const
{
  return createFrame(ranges,
                     this->minimalRange_,
                     this->maximalRange_,
                     0,
                     this->numberOfAzimutAngles_-1,
                     0,
                     this->numberOfElevationAngles_-1);
}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType> PointSet<PointType>
LIDAR3DFrameBuilder<PointType, RangeScalarType>::
  createFrame(const RangeVector & ranges,
              const double & userMinimalRange,
              const double & userMaximalRange)const
{
  return createFrame(ranges,
                     userMinimalRange,
                     userMaximalRange,
                     0,
                     this->numberOfAzimutAngles_-1,
                     0,
                     this->numberOfElevationAngles_-1);
}

//-----------------------------------------------------------------------------
template <class PointType, typename RangeScalarType> PointSet<PointType>
LIDAR3DFrameBuilder<PointType, RangeScalarType>::
  createFrame(const RangeVector & ranges,
              const double & userMinimalRange,
              const double & userMaximalRange,
              const size_t & userFirstAzimutAngleIndex,
              const size_t & userLastAzimutAngleIndex,
              const size_t & userFirstElevationAngleIndex,
              const size_t & userLastElevationAngleIndex)const
{
  assert(userFirstAzimutAngleIndex < this->numberOfAzimutAngles_);
  assert(userLastAzimutAngleIndex < this->numberOfAzimutAngles_);
  assert(userFirstElevationAngleIndex < this->numberOfElevationAngles_);
  assert(userLastElevationAngleIndex < this->numberOfElevationAngles_);

  size_t numberOfRanges = (userLastAzimutAngleIndex-userFirstAzimutAngleIndex)*
      (userLastElevationAngleIndex-userFirstElevationAngleIndex);

  typename LIDAR3DFrameBuilder<PointType, RangeScalarType>::Vector4 point;
  double minimalRange = std::max(this->minimalRange_, userMinimalRange);
  double maximalRange = std::min(this->maximalRange_, userMaximalRange);

  size_t rayIndex = 0;
  PointSet<PointType> points;
  points.reserve(numberOfRanges);

  if (this->scanStorageOrder_ == LIDAR3D::ScanStorageOrder::ELEVATION_MAJOR)
  {
    for (size_t i=userFirstAzimutAngleIndex; i<= userLastAzimutAngleIndex; i++ )
    {
      for (size_t j=userFirstElevationAngleIndex; j <= userLastElevationAngleIndex; j++, rayIndex++)
      {
        size_t rangeIndex = i*this->numberOfElevationAngles_+j;
        double d = ranges[rangeIndex];
        if (d > minimalRange && d < maximalRange)
        {
          point = this->H_*(this->rayUnitVectors_[rayIndex]*ranges[rangeIndex]+this->Zero_);
          points.emplace_back(point.template segment<PointTraits<PointType>::SIZE>(0));
        }
      }
    }
  } else {
    for (size_t j=userFirstElevationAngleIndex; j <= userLastElevationAngleIndex; j++)
    {
      for (size_t i=userFirstAzimutAngleIndex; i <= userLastAzimutAngleIndex; i++, rayIndex++)
      {
        size_t rangeIndex = j*this->numberOfAzimutAngles_+i;
        double d = ranges[rangeIndex];
        if (d > minimalRange && d < maximalRange)
        {
          point = this->H_*(this->rayUnitVectors_[rayIndex]*ranges[rangeIndex]+this->Zero_);
          points.emplace_back(point.template segment<PointTraits<PointType>::SIZE>(0));
        }
      }
    }
  }
  return points;
}


template class LIDAR3DFrameBuilder<Eigen::Vector2f, float>;
template class LIDAR3DFrameBuilder<Eigen::Vector2d, float>;
template class LIDAR3DFrameBuilder<Eigen::Vector3f, float>;
template class LIDAR3DFrameBuilder<Eigen::Vector3d, float>;
template class LIDAR3DFrameBuilder<Eigen::Vector2f, double>;
template class LIDAR3DFrameBuilder<Eigen::Vector2d, double>;
template class LIDAR3DFrameBuilder<Eigen::Vector3f, double>;
template class LIDAR3DFrameBuilder<Eigen::Vector3d, double>;

}  // namespace romea

