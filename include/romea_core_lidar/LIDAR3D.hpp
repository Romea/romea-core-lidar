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


#ifndef ROMEA_CORE_LIDAR__LIDAR3D_HPP_
#define ROMEA_CORE_LIDAR__LIDAR3D_HPP_

#include "romea_core_lidar/LIDAR2D.hpp"

namespace romea
{
namespace core
{

class LIDAR3D : public LIDAR2D
{
public:
  enum ScanStorageOrder
  {
    AZIMUT_MAJOR,
    ELEVATION_MAJOR,
  };

public:
  LIDAR3D(
    const double & frameRate,
    const double & minimalAzimutAngle,
    const double & maximalAzimutAngle,
    const double & azimutAngleIncrement,
    const double & azimutAngleStd,
    const double & minimalElevationAngle,
    const double & maximalElevationAngle,
    const double & elevationAngleIncrement,
    const double & elevationAngleStd,
    const double & mininalRange,
    const double & maximalRange,
    const double & rangeStd,
    const ScanStorageOrder & scanStorageOrder);

  virtual ~LIDAR3D() = default;

  const double & getMininalElevationAngle() const;
  const double & getMaximalElevationAngle() const;
  const double & getElevationAngleIncrement() const;
  const double & getElevationAngleStd() const;
  const double & getElevationAngleVariance() const;
  const double & getElevationAperture() const;

  const ScanStorageOrder & getScanStorageOrder()const;

protected:
  double minimalElevationAngle_;
  double maximalElevationAngle_;
  double elevationAngleIncrement_;
  double elevationAngleStd_;
  double elevationAngleVariance_;
  double elevationAperture_;

  ScanStorageOrder scanStorageOrder_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LIDAR__LIDAR3D_HPP_
