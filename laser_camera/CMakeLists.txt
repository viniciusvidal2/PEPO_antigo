cmake_minimum_required(VERSION 2.8.3)
project(laser_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs laser_geometry nav_msgs cv_bridge pcl_ros std_msgs tf2_ros tf tf_conversions camera_calibration_parsers camera_info_manager image_geometry laser_geometry
)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(../libraries/include)

catkin_package(CATKIN_DEPENDS)

add_executable(processar_node src/processar.cpp ../libraries/src/processcloud.cpp)
target_link_libraries(processar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
