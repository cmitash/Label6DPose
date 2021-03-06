cmake_minimum_required(VERSION 2.8.3)
project(label_poses)

find_package (PCL REQUIRED COMPONENTS io common visualization features) 

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  roslib 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  pcl_ros
  image_transport 
  image_geometry
  cv_bridge 
  message_generation 
  tf
)

catkin_package(
  LIBRARIES objectDetect
  INCLUDE include
  DEPENDS cv_bridge
  CATKIN_DEPENDS 
  image_transport 
  tf 
  geometry_msgs 
  image_geometry 
  pcl_ros
  roscpp
  roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(
			${PROJECT_NAME}_lib 
			${PROJECT_SOURCE_DIR}/label_poses.cpp)

target_link_libraries(${PROJECT_NAME}_lib 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} )

add_executable(label_poses ${PROJECT_SOURCE_DIR}/label_poses.cpp)
target_link_libraries(label_poses ${PROJECT_NAME}_lib)

