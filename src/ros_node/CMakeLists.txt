cmake_minimum_required(VERSION 3.12)
project(fugseg_node)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    fugseg
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(viewer
    src/viewer.cpp
    src/utils.cpp
)
target_link_libraries(viewer
    ${catkin_LIBRARIES}
)