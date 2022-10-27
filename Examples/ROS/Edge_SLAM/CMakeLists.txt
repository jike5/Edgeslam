cmake_minimum_required(VERSION 2.8.3)


# Edge-SLAM: replaced next include statement with the content of rosbuild.cmake
# and made edits as indicated below to prevent cmake deprecated policy warnings
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
########## Start of rosbuild.cmake ##########
#cmake_minimum_required(VERSION 2.4.6)

#
#  Catkin-compat thunks
#
# Edge-SLAM: added cmake version check if statement
if(${CMAKE_VERSION} VERSION_LESS "3.0")
  cmake_policy(SET CMP0011 OLD)
endif()

macro(rosbuild_catkinize)
endmacro()

# Edge-SLAM: added cmake version check if statement
# Policy settings to prevent warnings on 2.6 but ensure proper operation on
# 2.4.
if(COMMAND cmake_policy)
  if(${CMAKE_VERSION} VERSION_LESS "3.0")
    # Logical target names must be globally unique.
    cmake_policy(SET CMP0002 OLD)
    # Libraries linked via full path no longer produce linker search paths.
    cmake_policy(SET CMP0003 OLD)
    # Preprocessor definition values are now escaped automatically.
    cmake_policy(SET CMP0005 OLD)
    if(POLICY CMP0011)
      # Included scripts do automatic cmake_policy PUSH and POP.
      cmake_policy(SET CMP0011 OLD)
    endif(POLICY CMP0011)
  endif()
endif(COMMAND cmake_policy)

# Edge-SLAM: added to prevent cmake warning "Policy CMP0003 should be set before this line."
if(COMMAND cmake_policy)
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.0")
    cmake_policy(SET CMP0003 NEW)
  endif()
endif(COMMAND cmake_policy)

set(CMAKE_OSX_ARCHITECTURES "x86_64")

##############################################################################
# First things first: we must have rospack.  But only look at PATH, #3831
find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
if (NOT ROSPACK_EXE)
  message(FATAL_ERROR "Couldn't find rospack. Please run 'make' in $ROS_ROOT")
endif(NOT ROSPACK_EXE)
##############################################################################

# Load private macros (not to be used externally)
include($ENV{ROS_ROOT}/core/rosbuild/private.cmake)
# Load public macros (developer's API)
include($ENV{ROS_ROOT}/core/rosbuild/public.cmake)
########## End of rosbuild.cmake ##########


rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Vim https://github.com/Valloric/YouCompleteMe.git support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS system serialization filesystem REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libEdge_SLAM.so
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

