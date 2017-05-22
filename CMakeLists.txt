cmake_minimum_required(VERSION 2.8.3)
project(buoy_detect)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detect src/detect.cpp)
target_link_libraries(detect  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(pubs src/publisher.cpp)
target_link_libraries(pubs  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(subs src/subscriber.cpp)
target_link_libraries(subs  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
