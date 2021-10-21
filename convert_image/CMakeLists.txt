cmake_minimum_required(VERSION 3.0.2)
project(convert_image)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS sensor_msgs cv_bridge image_transport
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(convert_image src/convert_image.cpp)
target_link_libraries(convert_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
