cmake_minimum_required(VERSION 2.8.3)
project(laser_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs image_transport nav_msgs pcl_ros tf2_ros tf tf_conversions camera_calibration_parsers camera_info_manager image_geometry laser_geometry)
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

catkin_package(CATKIN_DEPENDS DEPENDS roscpp cv_bridge sensor_msgs std_msgs image_transport nav_msgs pcl_ros tf2_ros tf tf_conversions camera_calibration_parsers camera_info_manager image_geometry laser_geometry)

#add_executable(processar_node src/processar.cpp ../libraries/src/processcloud.cpp)
#target_compile_options(processar_node PRIVATE ${OpenMP_FLAGS})
#add_dependencies(processar_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(processar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

#add_executable(processar_online_node src/processar_online.cpp ../libraries/src/processcloud.cpp)
#target_compile_options(processar_online_node PRIVATE ${OpenMP_FLAGS})
#add_dependencies(processar_online_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(processar_online_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

#add_executable(otimizar_nuvem_node src/otimizar_nuvem.cpp ../libraries/src/processcloud.cpp)
#target_compile_options(otimizar_nuvem_node PRIVATE ${OpenMP_FLAGS})
#add_dependencies(otimizar_nuvem_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(otimizar_nuvem_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(clusters_node src/separar_clusters.cpp ../libraries/src/clusters.cpp ../libraries/src/processcloud.cpp)
target_compile_options(clusters_node PRIVATE ${OpenMP_FLAGS})
add_dependencies(clusters_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(clusters_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

#add_executable(otimiza_manual src/otimiza_manual.cpp ../libraries/src/clusters.cpp ../libraries/src/processcloud.cpp ../libraries/src/otimizaimagens.cpp)
#target_compile_options(otimiza_manual PRIVATE ${OpenMP_FLAGS})
#add_dependencies(otimiza_manual ${catkin_EXPORTED_TARGETS})
#target_link_libraries(otimiza_manual ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

#add_executable(otimiza_bat src/otimiza_bat.cpp ../libraries/src/processcloud.cpp ../libraries/src/otimizaimagens.cpp)
#target_compile_options(otimiza_bat PRIVATE ${OpenMP_FLAGS})
#add_dependencies(otimiza_bat ${catkin_EXPORTED_TARGETS})
#target_link_libraries(otimiza_bat ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})

#add_executable(scanner_aquisicao src/scanner_aquisicao.cpp ../libraries/src/processcloud.cpp)
#target_compile_options(scanner_aquisicao PRIVATE ${OpenMP_FLAGS})
#add_dependencies(scanner_aquisicao ${catkin_EXPORTED_TARGETS})
#target_link_libraries(scanner_aquisicao ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_LIBS})
