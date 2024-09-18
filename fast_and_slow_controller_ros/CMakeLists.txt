cmake_minimum_required(VERSION 2.8.3)
project(fast_and_slow_controller_ros)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport cv_bridge rospy)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(fast_and_slow_controller src/fast_and_slow_controller.cpp)
target_link_libraries(fast_and_slow_controller ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)