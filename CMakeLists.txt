cmake_minimum_required(VERSION 2.8.3)
project(romea_lidar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17 -O3")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS romea_common)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(Boost REQUIRED COMPONENTS system)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})


###########
## Build ##
###########
include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
    )

add_library(romea_lidar
    src/LIDAR2D.cpp
    src/LIDAR2DFrameBuilder.cpp
    src/LIDAR2DFrameBuilderBase.cpp
    src/LIDAR2DUndistortedFrameBuilder.cpp
    src/LIDAR3D.cpp
    src/LIDAR3DFrameBuilder.cpp
    src/LIDAR3DFrameBuilderBase.cpp
    src/LIDAR3DUndistortedFrameBuilder.cpp
    src/LIDARPose.cpp
    )

