cmake_minimum_required(VERSION 3.12)
project(fugseg)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fugseg
)

include_directories(include)

add_library(fugseg
    src/fugseg.cpp
    src/dataset.cpp
    src/cloud.cpp
    src/utils.cpp
)
target_link_libraries(fugseg
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
