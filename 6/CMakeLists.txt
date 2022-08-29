cmake_minimum_required(VERSION 3.20)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

set(BUNNY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models/bunny/bunny.obj")

set(DEMO_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Assignment6.pdf
    ${BUNNY_DIR}
)

set(SRCS
    main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp
    Triangle.hpp Scene.cpp Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp
    Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp Renderer.cpp Renderer.hpp
)

add_executable(RayTracing ${SRCS} ${DEMO_RESOURCES})

source_group(src FILES ${SRCS})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${DEMO_RESOURCES})

target_compile_definitions(RayTracing PRIVATE
    BUNNY_DIR="${BUNNY_DIR}"
)
