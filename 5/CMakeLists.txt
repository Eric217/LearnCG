cmake_minimum_required(VERSION 3.20)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)
set(SRCS
    main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp 
    Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp
)

add_executable(RayTracing ${SRCS})
target_compile_options(RayTracing PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
target_link_libraries(RayTracing PUBLIC -fsanitize=undefined)
 
source_group(src FILES ${SRCS})
