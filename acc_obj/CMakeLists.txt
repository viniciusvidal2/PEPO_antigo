cmake_minimum_required(VERSION 2.8.3)
project(acc_obj)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs nav_msgs pcl_ros)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(../libraries/include)
include_directories(${OpenMP_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS DEPENDS roscpp cv_bridge sensor_msgs std_msgs nav_msgs pcl_ros)

add_executable(acc_cloud_node src/acc_cloud.cpp ../libraries/src/processcloud.cpp)
target_compile_options(acc_cloud_node PRIVATE ${OpenMP_FLAGS})
add_dependencies(acc_cloud_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(acc_cloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(acc_obj_node src/acc_obj.cpp ../libraries/src/registerobjectoptm.cpp ../libraries/src/processcloud.cpp)
target_compile_options(acc_obj_node PRIVATE ${OpenMP_FLAGS})
add_dependencies(acc_obj_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(acc_obj_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})
