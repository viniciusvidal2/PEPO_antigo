cmake_minimum_required(VERSION 2.8.3)
project(laser_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs laser_geometry nav_msgs cv_bridge pcl_ros std_msgs tf2_ros tf tf_conversions camera_calibration_parsers camera_info_manager image_geometry laser_geometry
)
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

catkin_package(CATKIN_DEPENDS DEPENDS OpenMP)

add_executable(processar_node src/processar.cpp ../libraries/src/processcloud.cpp)
target_link_libraries(processar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(otimizar_nuvem_node src/otimizar_nuvem.cpp ../libraries/src/processcloud.cpp)
target_link_libraries(otimizar_nuvem_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(clusters_node src/separar_clusters.cpp ../libraries/src/clusters.cpp ../libraries/src/processcloud.cpp)
target_link_libraries(clusters_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(processa_imagens src/processa_imagens.cpp ../libraries/src/otimizaimagens.cpp)
target_link_libraries(processa_imagens ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})
