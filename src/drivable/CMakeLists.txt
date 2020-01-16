cmake_minimum_required(VERSION 3.1.5)
project(drivable)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED sensor_msgs nav_msgs nodelet openslam_gmapping roscpp tf)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES utils sensor_base sensor_odometry sensor_range log configfile scanmatcher gridfastsla
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED common io)

list (REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
message(STATUS "${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}")
add_executable(drivable main.cpp CameraModel.h ImageManage.h Map.h)
target_link_libraries(drivable ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})