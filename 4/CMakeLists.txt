cmake_minimum_required (VERSION 3.20.0)

project(BezierCurve)

set(CMAKE_MODULE_PATH
     /usr/local/share/cmake/Modules
     /usr/local/lib/cmake
)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(SRCS
    main.cpp
)
add_executable(BezierCurve ${SRCS})

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})

source_group(src FILES ${SRCS} )
