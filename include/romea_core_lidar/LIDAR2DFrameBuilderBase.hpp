// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_CORE_LIDAR__LIDAR2DFRAMEBUILDERBASE_HPP_
#define ROMEA_CORE_LIDAR__LIDAR2DFRAMEBUILDERBASE_HPP_

// romea
#include "romea_core_lidar/LIDAR2D.hpp"

namespace romea
{

template<typename Scalar>
class LIDAR2DFrameBuilderBase
{
public:
  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

public:
  LIDAR2DFrameBuilderBase();

  explicit LIDAR2DFrameBuilderBase(const LIDAR2D & lidar);

  virtual ~LIDAR2DFrameBuilderBase() = default;

  virtual void init(const LIDAR2D & lidar);

protected:
  Matrix4 H_;
  Vector4 Zero_;
  double minimalRange_;
  double maximalRange_;

  PointSet<Vector4> rayUnitVectors_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDAR2DFRAMEBUILDERBASE_HPP_
