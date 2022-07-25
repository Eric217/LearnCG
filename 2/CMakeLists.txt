cmake_minimum_required (VERSION 3.20.0)
set(TARGET_NAME Rasterizer2)
project (${TARGET_NAME})
 
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEMO_RESOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/Assignment2.pdf
)
add_executable(${TARGET_NAME} main.cpp rasterizer.hpp rasterizer.cpp 
    global.hpp Triangle.hpp Triangle.cpp ${DEMO_RESOURCES})

target_include_directories(${TARGET_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${TARGET_NAME} PRIVATE ${OpenCV_LIBS})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${DEMO_RESOURCES})
