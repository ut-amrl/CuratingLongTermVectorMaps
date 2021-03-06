PROJECT(curating_long_term_vector_maps)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Curating Long Term Vector Maps Implementation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

IF (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9)
    SET(CMAKE_CXX_FLAGS "-Wno-deprecated-copy")
ENDIF()

IF (${CMAKE_BUILD_TYPE} MATCHES "Release")
    option(TESTING "Build tests" OFF)
    MESSAGE(STATUS "Additional Flags for Release mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    option(TESTING "Build tests" ON)
    MESSAGE(STATUS "Additional Flags for Debug mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O0")
ENDIF()

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

IF (VECTOR_MAPS_LIBRARY)
    SET(libs gflags glog boost_system m X11)
    ADD_LIBRARY(vector_maps src/line_extraction.cc)
    TARGET_LINK_LIBRARIES(vector_maps ${libs})
    SET_TARGET_PROPERTIES(vector_maps PROPERTIES
                          COMPILE_FLAGS ${CMAKE_CXX_FLAGS})
    MESSAGE(STATUS "Building Vector Maps as a library")
    MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
ELSE()
    FIND_PACKAGE(PCL REQUIRED)
    INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
    INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
    
    rosbuild_init()
    
    SET(ROS_BUILD_STATIC_LIBS true)
    SET(ROS_BUILD_SHARED_LIBS false)
    
    MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
    MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
    
    rosbuild_genmsg()
    
    SET(libs roslib roscpp rosbag console_bridge cv_bridge gflags glog boost_filesystem boost_system m X11)
    
    ROSBUILD_ADD_EXECUTABLE(stf_filter
                            src/stf_filter.cc
                            src/SDFTable.cc
                            src/line_extraction.cc)
                            
    TARGET_LINK_LIBRARIES(stf_filter
                          ${libs}
                          ${CERES_LIBRARIES})
ENDIF()

IF (TESTING)
  enable_testing()
  add_subdirectory(test)
ENDIF()
