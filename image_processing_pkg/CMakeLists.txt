cmake_minimum_required(VERSION 3.0.2)
project(image_processing_pkg)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_publisher_node src/image_publisher_node.cpp)
add_executable(image_processor_node src/image_processor_node.cpp)
target_link_libraries(image_processor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(image_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
