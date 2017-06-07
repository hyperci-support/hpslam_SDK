cmake_minimum_required(VERSION 2.8)
project( hpslam )

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


#include header files
include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/Sensor_Binocular
${PROJECT_SOURCE_DIR}/viewer
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Sensor_Binocular/lib/libSensor.so
${PROJECT_SOURCE_DIR}/viewer/lib/libViewer.so
libusb-1.0.so
libcyusb.so
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#add .cpp files
#set(SOURCES src/main.cpp src/Camera.cpp src/CamPose.cpp src/Feature.cpp src/EKFSystem.cpp src/PangoViewer.cpp src/IMUFilter.cpp)

#add_executable( aslam ${SOURCES} )
#target_link_libraries( aslam ${LIBS} )

#add_library(${PROJECT_NAME} SHARED
#src/hpslam.cpp 
#src/Camera.cpp 
#src/CamPose.cpp 
#src/Feature.cpp 
#src/EKFSystem.cpp 
#src/PangoViewer.cpp 
#src/IMUFilter.cpp
#)
#target_link_libraries( ${PROJECT_NAME} ${LIBS} )

add_executable( testhpslam testhpslam.cpp)
target_link_libraries( testhpslam ${LIBS} ${PROJECT_SOURCE_DIR}/lib/libhpslam.so)

#add_executable( aslamd src/maindelay1.cpp src/Camera.cpp src/CamPose.cpp src/Feature.cpp src/EKFSystem.cpp src/PangoViewer.cpp src/IMUFilter.cpp)
#target_link_libraries( aslamd ${LIBS} )

#add_executable( aslamd2 src/maindelay2.cpp src/Camera.cpp src/CamPose.cpp src/Feature.cpp src/EKFSystem.cpp src/PangoViewer.cpp src/IMUFilter.cpp)
#target_link_libraries( aslamd2 ${LIBS} )

#add_executable( test src/test.cc )
#target_link_libraries( test ${LIBS} )

#add_executable( imufusion src/IMUTest.cpp src/Camera.cpp src/CamPose.cpp src/Feature.cpp src/EKFSystem.cpp src/PangoViewer.cpp src/IMUFilter.cpp)
#target_link_libraries( imufusion ${LIBS} )

#add_executable( recordslow src/recordslow.cpp src/Camera.cpp src/CamPose.cpp src/Feature.cpp src/EKFSystem.cpp src/PangoViewer.cpp src/IMUFilter.cpp)
#target_link_libraries( recordslow ${LIBS} )

#add_executable( accelstatic src/accelstatic.cpp src/Camera.cpp src/CamPose.cpp src/Feature.cpp src/EKFSystem.cpp src/PangoViewer.cpp src/IMUFilter.cpp)
#target_link_libraries( accelstatic ${LIBS} )
