cmake_minimum_required(VERSION 2.8.3)
project(apriltags)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules
)

find_package(Eigen3 QUIET)
if(NOT Eigen_FOUND)
  set(Eigen_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
endif()

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltags
  CATKIN_DEPENDS
  DEPENDS Eigen OpenCV
)

include_directories(include include/AprilTags)
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if(ODROID_CROSS_COMPILE)
    set(C11_COMP_FLAGS "-W -pipe -O3 -mcpu=cortex-a15.cortex-a7 -mfpu=vfpv3-d16-fp16 -mfloat-abi=hard")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C11_COMP_FLAGS}")
else()
    if(JETSON_CROSS_COMPILE)
    set(C11_COMP_FLAGS "-W -pipe -Ofast -mcpu=cortex-a57 -std=c++11 -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C11_COMP_FLAGS}")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pipe -Ofast -std=c++11 -fopenmp")
endif()
endif()

add_library(apriltags
  src/Edge.cc src/FloatImage.cc src/GLine2D.cc src/GLineSegment2D.cc src/Gaussian.cc src/GrayModel.cc src/Homography33.cc src/MathUtil.cc src/Quad.cc src/Segment.cc src/TagDetection.cc src/TagDetector.cc src/TagFamily.cc src/UnionFindSimple.cc
)
target_link_libraries(apriltags
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS apriltags
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
