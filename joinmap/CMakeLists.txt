cmake_minimum_required(VERSION 3.10)
project(joinmap)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PCL REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(joinmap joinmap.cpp)
target_link_libraries(joinmap ${OpenCV_LIBS} ${PCL_LIBRARIES})