cmake_minimum_required (VERSION 3.20.0)
set(TARGET_NAME Rasterizer)
project (${TARGET_NAME})
 
set(CMAKE_MODULE_PATH 
    /usr/local/share/cmake/Modules
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

#message(STATUS "CV: ${OPENCV_LIBRARIES}")

set(CMAKE_CXX_STANDARD 17)

add_executable(${TARGET_NAME} main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

target_include_directories(${TARGET_NAME} PRIVATE ${OPENCV_INCLUDE_DIR}  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${TARGET_NAME} PRIVATE ${OPENCV_LIBRARIES})