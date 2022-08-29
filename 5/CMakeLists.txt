cmake_minimum_required(VERSION 3.20)
set(TARGET_NAME WhittedRayTracing)
project(${TARGET_NAME})

set(CMAKE_CXX_STANDARD 17)
set(SRCS
    main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp 
    Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp
)

add_executable(${TARGET_NAME} ${SRCS})
target_compile_options(${TARGET_NAME} PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
target_link_libraries(${TARGET_NAME} PUBLIC -fsanitize=undefined)
 
source_group(src FILES ${SRCS})
