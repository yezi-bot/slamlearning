cmake_minimum_required(VERSION 3.10)
project(loop_closure)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )
# add_executable( feature_training feature_training.cpp )
# target_link_libraries( feature_training ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( loop_closure loop_closure.cpp )
target_link_libraries( loop_closure ${OpenCV_LIBS} ${DBoW3_LIBS} )

# add_executable( gen_vocab gen_vocab_large.cpp )
# target_link_libraries( gen_vocab ${OpenCV_LIBS} ${DBoW3_LIBS} )