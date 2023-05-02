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


// romea core
#include <romea_core_common/pointset/PointTraits.hpp>

// std
#include <algorithm>
#include <limits>

// local
#include "romea_core_lidar/LIDAR3DFrameBuilderBase.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
template<typename Scalar>
LIDAR3DFrameBuilderBase<Scalar>::LIDAR3DFrameBuilderBase()
: H_(),
  Zero_(0, 0, 0, 1),
  minimalRange_(0),
  maximalRange_(std::numeric_limits<Scalar>::max()),
  numberOfAzimutAngles_(0),
  numberOfElevationAngles_(0),
  rayUnitVectors_()
{
}


//-----------------------------------------------------------------------------
template<typename Scalar>
LIDAR3DFrameBuilderBase<Scalar>::LIDAR3DFrameBuilderBase(const LIDAR3D & lidar)
{
  init(lidar);
}

//-----------------------------------------------------------------------------
template<typename Scalar>
void LIDAR3DFrameBuilderBase<Scalar>::init(const LIDAR3D & lidar)
{
  H_ = lidar.getBodyPose().matrix().cast<Scalar>();
  minimalRange_ = lidar.getMinimalRange();
  maximalRange_ = lidar.getMaximalRange();

  numberOfAzimutAngles_ = size_t(lidar.getAzimutAperture() / lidar.getAzimutAngleIncrement()) + 1;
  numberOfElevationAngles_ = size_t(
    lidar.getElevationAperture() / lidar.getElevationAngleIncrement()) + 1;
  scanStorageOrder_ = lidar.getScanStorageOrder();

  rayUnitVectors_.resize(numberOfAzimutAngles_ * numberOfElevationAngles_, Vector4(0, 0, 0, 0));
  if (scanStorageOrder_ == LIDAR3D::ScanStorageOrder::ELEVATION_MAJOR) {
    for (size_t i = 0; i < numberOfAzimutAngles_; i++) {
      Scalar azimutAngle = lidar.getMinimalAzimutAngle() + i * lidar.getAzimutAngleIncrement();
      Scalar cosAzimut = std::cos(azimutAngle);
      Scalar sinAzimut = std::sin(azimutAngle);

      for (size_t j = 0; j < numberOfElevationAngles_; j++) {
        Scalar elevationAngle = lidar.getMininalElevationAngle() +
          j * lidar.getElevationAngleIncrement();

        size_t k = i * numberOfElevationAngles_ + j;
        rayUnitVectors_[k](0) = std::cos(elevationAngle) * sinAzimut;
        rayUnitVectors_[k](1) = std::cos(elevationAngle) * cosAzimut;
        rayUnitVectors_[k](2) = std::sin(elevationAngle);
      }
    }
  } else {
    for (size_t i = 0; i < numberOfElevationAngles_; i++) {
      Scalar elevationAngle = lidar.getMininalElevationAngle() +
        i * lidar.getElevationAngleIncrement();
      Scalar cosElevation = std::cos(elevationAngle);
      Scalar sinElevation = std::sin(elevationAngle);

      for (size_t j = 0; j < numberOfAzimutAngles_; j++) {
        Scalar azimutAngle = lidar.getMinimalAzimutAngle() + j * lidar.getAzimutAngleIncrement();

        size_t k = i * numberOfAzimutAngles_ + j;
        rayUnitVectors_[k](0) = cosElevation * sin(azimutAngle);
        rayUnitVectors_[k](1) = cosElevation * cos(azimutAngle);
        rayUnitVectors_[k](2) = sinElevation;
      }
    }
  }
}

template class LIDAR3DFrameBuilderBase<float>;
template class LIDAR3DFrameBuilderBase<double>;

}  // namespace romea
