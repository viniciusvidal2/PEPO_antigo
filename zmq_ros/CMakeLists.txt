cmake_minimum_required(VERSION 2.8.3)
project(zmq_ros)

add_compile_options(-std=c++11)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs image_transport pcl_ros image_geometry tf2_ros pcl_conversions)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${ZeroMQ_INCLUDE_DIR})
include_directories(${OpenMP_INCLUDE_DIRS})

catkin_package(
  DEPENDS roscpp cv_bridge sensor_msgs std_msgs image_transport pcl_ros image_geometry tf2_ros pcl_conversions
)

add_library(protos msgs/imagem.pb.cc msgs/nuvem.pb.cc msgs/arquivos.pb.cc msgs/nvm.pb.cc)

add_executable(recebe_nuvem src/recebe_nuvem.cpp)
target_compile_options(recebe_nuvem PRIVATE ${OpenMP_FLAGS})
add_dependencies(recebe_nuvem ${catkin_EXPORTED_TARGETS})
target_link_libraries(recebe_nuvem ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES} protos ${ZeroMQ_LIBRARY} ${OpenMP_LIBS})

add_executable(recebe_imagem src/recebe_imagem.cpp)
target_compile_options(recebe_imagem PRIVATE ${OpenMP_FLAGS})
add_dependencies(recebe_imagem ${catkin_EXPORTED_TARGETS})
target_link_libraries(recebe_imagem ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES} ${ZeroMQ_LIBRARY} ${OpenMP_LIBS})

add_executable(recebe_dados_zmq src/recebe_dados_zmq.cpp)
target_compile_options(recebe_dados_zmq PRIVATE ${OpenMP_FLAGS})
add_dependencies(recebe_dados_zmq ${catkin_EXPORTED_TARGETS})
target_link_libraries(recebe_dados_zmq ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES} protos ${ZeroMQ_LIBRARY} ${OpenMP_LIBS})
