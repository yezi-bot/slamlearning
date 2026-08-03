cmake_minimum_required(VERSION 3.10)
project(direct)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(fmt REQUIRED)

include_directories(
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
)

add_executable(direct direct.cpp)
target_link_libraries(direct 
Eigen3::Eigen
${OpenCV_LIBS}
g2o_core
g2o_stuff
g2o_types_slam2d
g2o_types_slam3d
g2o_solver_csparse
g2o_csparse_extension
g2o_solver_dense
g2o_solver_pcg
g2o_types_sba                   //新版要这么写
fmt
)