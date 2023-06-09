cmake_minimum_required(VERSION 3.5)

project(
  "romea_core_lidar"
  VERSION 0.1.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(romea_core_common REQUIRED)

add_library(${PROJECT_NAME} SHARED
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

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${romea_core_common_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3 -std=c++17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${romea_core_common_LIBRARIES} tbb)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

#if(BUILD_TESTING)
#  enable_testing()
#  add_subdirectory(test)
#endif(BUILD_TESTING)
