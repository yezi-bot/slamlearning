cmake_minimum_required(VERSION 3.10)
project(main)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )
add_executable(main main.cpp) 
target_link_libraries(main ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen)