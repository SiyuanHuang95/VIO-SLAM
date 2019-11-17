cmake_minimum_required(VERSION 2.8.3)
project(vio)

find_package(OpenCV 3.3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(rotation_comparsion week1_introduction_math/src/rotation_comparsion.cpp)
target_link_libraries(rotation_comparsion ${Sophus_LIBRARIES})
