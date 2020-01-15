cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)

option (USE_CUDA "Use cuda for color conversion (only yuyv2rgb supported)" ON) 
message(STATUS "USE_CUDA is ${USE_CUDA}")
if (USE_CUDA)
    add_definitions(-DUSE_CUDA)
endif()

add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS "-fPIC")
SET(CMAKE_C_FLAGS "-fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)
find_package(OpenCV REQUIRED)
#find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager cv_bridge)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
if (USE_CUDA)
    find_package(CUDA REQUIRED)
endif(USE_CUDA)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

if (USE_CUDA)
    include_directories(include
        ${CUDA_INCLUDE_DIRS}
        )
endif(USE_CUDA)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp src/color_conversion.cpp)
set(external_libraries
    ${avcodec_LIBRARIES}
    ${swscale_LIBRARIES}
    ${catkin_LIBRARIES}
  )

if (USE_CUDA)
    add_subdirectory(src/cuda src/cuda/lib)
    set(EXTRA_LIBS ${EXTRA_LIBS} color_conversion_cuda)
    set(external_libraries
        ${external_libraries}
        ${EXTRA_LIBS}
        ${CUDA_LIBS}
        )
endif(USE_CUDA)

target_link_libraries(${PROJECT_NAME}
  ${external_libraries}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
add_executable(${PROJECT_NAME}_trimple_node nodes/usb_cam_trimple_node.cpp)
add_executable(${PROJECT_NAME}_receiver_node nodes/receiver_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_trimple_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_receiver_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_receiver_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
