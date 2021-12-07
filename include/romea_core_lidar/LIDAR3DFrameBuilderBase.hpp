#ifndef romea_LIDAR3DFrameBuilderBase_hpp
#define romea_LIDAR3DFrameBuilderBase_hpp

//romea
#include <romea_core_common/time/Time.hpp>
#include "LIDAR3D.hpp"

namespace romea {


template <typename Scalar>
class LIDAR3DFrameBuilderBase
{

public :

  using Vector4 =Eigen::Matrix<Scalar,4,1> ;
  using Matrix4 =Eigen::Matrix<Scalar,4,4> ;

public :

  LIDAR3DFrameBuilderBase();

  LIDAR3DFrameBuilderBase(const LIDAR3D & lidar);

  virtual ~LIDAR3DFrameBuilderBase()=default;

  void init(const LIDAR3D & lidar);

protected :

  Matrix4 H_;
  Vector4 Zero_;

  double minimalRange_;
  double maximalRange_;
  size_t numberOfAzimutAngles_;
  size_t numberOfElevationAngles_;
  LIDAR3D::ScanStorageOrder scanStorageOrder_;

  PointSet<Vector4> rayUnitVectors_;

};

}
#endif
