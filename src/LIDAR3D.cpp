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


#include "romea_core_lidar/LIDAR3D.hpp"

namespace romea
{
namespace core
{

LIDAR3D::LIDAR3D(
  const double & rate,
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
  const ScanStorageOrder & scanStorageOrder)
: LIDAR2D(rate,
    minimalAzimutAngle,
    maximalAzimutAngle,
    azimutAngleIncrement,
    azimutAngleStd,
    mininalRange,
    maximalRange,
    rangeStd),
  minimalElevationAngle_(minimalElevationAngle),
  maximalElevationAngle_(maximalElevationAngle),
  elevationAngleIncrement_(elevationAngleIncrement),
  elevationAngleStd_(elevationAngleStd),
  elevationAngleVariance_(elevationAngleStd * elevationAngleStd),
  elevationAperture_(maximalElevationAngle - minimalElevationAngle),
  scanStorageOrder_(scanStorageOrder)
{
}

//-----------------------------------------------------------------------------
const double & LIDAR3D::getMininalElevationAngle() const
{
  return minimalElevationAngle_;
}

//-----------------------------------------------------------------------------
const double & LIDAR3D::getMaximalElevationAngle() const
{
  return maximalElevationAngle_;
}

//-----------------------------------------------------------------------------
const double & LIDAR3D::getElevationAngleIncrement() const
{
  return elevationAngleIncrement_;
}

//-----------------------------------------------------------------------------
const double & LIDAR3D::getElevationAngleStd() const
{
  return elevationAngleStd_;
}

//-----------------------------------------------------------------------------
const double & LIDAR3D::getElevationAngleVariance() const
{
  return elevationAngleVariance_;
}

//-----------------------------------------------------------------------------
const double & LIDAR3D::getElevationAperture() const
{
  return elevationAperture_;
}


//-----------------------------------------------------------------------------
const LIDAR3D::ScanStorageOrder & LIDAR3D::getScanStorageOrder()const
{
  return scanStorageOrder_;
}

}  // namespace core
}  // namespace romea
