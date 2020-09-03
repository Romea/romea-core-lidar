#ifndef romea_LIDARFrameFeatures_hpp
#define romea_LIDARFrameFeatures_hpp

//romea
#include "romea_common/pointset/PointSet.hpp"
#include "romea_common/pointset/NormalSet.hpp"
#include "romea_common/pointset/KdTree.hpp"

namespace romea {

template <class PointType>
struct LIDARFrameFeatures
{
  using Ptr = std::shared_ptr<LIDARFrameFeatures<PointType> > ;
  using ConstPtr =std::shared_ptr<const LIDARFrameFeatures<PointType> > ;

  KdTree<PointType> kdTree;
  NormalSet<PointType> normals;
  std::vector<double> curvatures;
  std::vector<double> pointsNormalsReliability;
};

}

#endif
