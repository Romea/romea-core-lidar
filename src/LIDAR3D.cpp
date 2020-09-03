#include "romea_lidar/LIDAR3D.hpp"

namespace romea
{

LIDAR3D::LIDAR3D(const double & rate,
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
                 const ScanStorageOrder &scanStorageOrder):
  LIDAR2D(rate,
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
  elevationAngleVariance_(elevationAngleStd*elevationAngleStd),
  elevationAperture_(maximalElevationAngle-minimalElevationAngle),
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
const LIDAR3D::ScanStorageOrder &LIDAR3D::getScanStorageOrder()const
{
  return scanStorageOrder_;
}


}
