cmake_minimum_required (VERSION 3.20.0)
set(TARGET_NAME Rasterizer3)

project (${TARGET_NAME})
 
set(CMAKE_MODULE_PATH
    /usr/local/share/cmake/Modules
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
 
set(CMAKE_CXX_STANDARD 17)
file(GLOB_RECURSE DEMO_MODELS ./models/*)

set(DEMO_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Assignment3.pdf
    ${DEMO_MODELS}
)
set(SRCS
    main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp
    Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h
)

add_executable(${TARGET_NAME} ${SRCS} ${DEMO_RESOURCES})
target_link_libraries(Rasterizer3 ${OpenCV_LIBRARIES})
target_include_directories(${TARGET_NAME} PRIVATE ${OPENCV_INCLUDE_DIR}  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${TARGET_NAME} PRIVATE ${OPENCV_LIBRARIES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${DEMO_RESOURCES})
source_group(src FILES ${SRCS} )

