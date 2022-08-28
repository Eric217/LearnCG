cmake_minimum_required(VERSION 3.20)
project(BezierCurve)

set(CMAKE_MODULE_PATH
     /usr/local/share/cmake/Modules
     /usr/local/lib/cmake
)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})

set(SRCS
    main.cpp  
)
source_group(src FILES ${SRCS} )
