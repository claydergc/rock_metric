cmake_minimum_required(VERSION 2.8.3)
project(rock_metric)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  OpenCV
  cv_bridge
  image_transport)

#find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.4.0 REQUIRED)
find_package(Qt5 REQUIRED Widgets)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
#	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs cv_bridge image_transport
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

#set(project_SOURCES main.cpp pclviewer.cpp)
set(project_SOURCES src/rock_metric.cpp src/ui/UIMain.hpp src/ui/UIDfragSmart.hpp)
#set(project_SOURCES src/rock_metric.cpp src/UIMain.h)
add_executable(${PROJECT_NAME} ${project_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} Qt5::Widgets)
#target_link_libraries(rock_metric ${catkin_LIBRARIES} ${PCL_LIBRARIES} Qt5::Widgets)
