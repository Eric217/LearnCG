cmake_minimum_required (VERSION 3.20.0)
set(TARGET_NAME Transformation)
project (${TARGET_NAME})

 
find_package(Eigen3 REQUIRED)

add_executable (${TARGET_NAME} main.cpp)

target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
