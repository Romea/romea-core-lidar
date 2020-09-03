#ifndef romea_LIDAR3D_hpp
#define romea_LIDAR3D_hpp

#include "LIDAR2D.hpp"

namespace romea
{

 class LIDAR3D : public LIDAR2D
 {

 public :

   enum ScanStorageOrder
   {
     AZIMUT_MAJOR,
     ELEVATION_MAJOR,
   };

 public:

   LIDAR3D(const double & frameRate,
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

   virtual ~LIDAR3D()=default;

   const double & getMininalElevationAngle() const;
   const double & getMaximalElevationAngle() const;
   const double & getElevationAngleIncrement() const;
   const double & getElevationAngleStd() const;
   const double & getElevationAngleVariance() const;
   const double & getElevationAperture() const;

   const ScanStorageOrder & getScanStorageOrder()const;

 protected :

   double minimalElevationAngle_;
   double maximalElevationAngle_;
   double elevationAngleIncrement_;
   double elevationAngleStd_;
   double elevationAngleVariance_;
   double elevationAperture_;

   ScanStorageOrder scanStorageOrder_;

 };

}

#endif
