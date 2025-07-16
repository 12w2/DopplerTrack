cmake_minimum_required(VERSION 3.0.2)
project(dopplertrack)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
)

find_package(OpenCV REQUIRED) 


add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)


set(DETECTING_SOURCES
  src/detecting/CVC_cluster.cpp
  src/detecting/lshaped_fitting.cpp
)



# publisher_node
add_executable(publisher_node src/publisher_node.cpp)
target_link_libraries(publisher_node
  ${catkin_LIBRARIES}
)
add_dependencies(publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# detector_node
add_executable(detector_node
  src/detector_node.cpp
  ${DETECTING_SOURCES}           
)
target_link_libraries(detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}               
)
add_dependencies(detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# tracker_node
add_executable(tracker_node
  src/tracker_node.cpp
  ${DETECTING_SOURCES}          
)
target_link_libraries(tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}              
)
add_dependencies(tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})