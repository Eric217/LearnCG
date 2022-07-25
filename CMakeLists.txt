cmake_minimum_required (VERSION 3.20.0)

project(CG)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH
     /usr/local/share/cmake/Modules
     /usr/local/lib/cmake
)

add_subdirectory(./0)
add_subdirectory(./1)
add_subdirectory(./2)
add_subdirectory(./3)
