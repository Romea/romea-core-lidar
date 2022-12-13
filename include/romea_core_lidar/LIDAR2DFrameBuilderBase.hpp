#ifndef ROMEA_CORE_LIDAR_LIDAR2DFRAMEBUILDERBASE_HPP_
#define ROMEA_CORE_LIDAR_LIDAR2DFRAMEBUILDERBASE_HPP_

// romea
#include "romea_core_lidar/LIDAR2D.hpp"

namespace romea {

template <typename Scalar>
class LIDAR2DFrameBuilderBase
{
public :
  using Vector4 = Eigen::Matrix<Scalar, 4, 1> ;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4> ;

public :
  LIDAR2DFrameBuilderBase();

  explicit LIDAR2DFrameBuilderBase(const LIDAR2D & lidar);

  virtual ~LIDAR2DFrameBuilderBase() = default;

  virtual void init(const LIDAR2D & lidar);

protected :

  Matrix4 H_;
  Vector4 Zero_;
  double minimalRange_;
  double maximalRange_;

  PointSet<Vector4> rayUnitVectors_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR_LIDAR2DFRAMEBUILDERBASE_HPP_
