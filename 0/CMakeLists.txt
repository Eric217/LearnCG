cmake_minimum_required (VERSION 3.20.0)
set(TARGET_NAME Transformation)
project (${TARGET_NAME})


set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    /usr/local/Cellar/eigen/3.3.7/share/cmake/Modules
)

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
    message(STATUS ${EIGEN3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "library not found")
endif()


add_executable (${TARGET_NAME} main.cpp)

target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
