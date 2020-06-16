##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(programinha)

add_compile_options(-std=c++11)

find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS qt_build roscpp sensor_msgs pcl_ros cv_bridge)
find_package(Protobuf REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${ZeroMQ_INCLUDE_DIR})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/programinha/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Libraries
##############################################################################

FIND_LIBRARY(libssh_LIBRARIES libssh.so /usr/lib/x86_64-linux-gnu)

add_library(protos msgs/imagem.pb.cc msgs/nuvem.pb.cc msgs/arquivos.pb.cc msgs/nvm.pb.cc)

##############################################################################
# Binaries
##############################################################################

add_executable(programinha ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(programinha ${QT_LIBRARIES} ${catkin_LIBRARIES} ${libssh_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PROTOBUF_LIBRARIES} protos ${ZeroMQ_LIBRARY})
install(TARGETS programinha RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

