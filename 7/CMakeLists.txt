cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME PathTracing)

project(${TARGET_NAME})

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE DEMO_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/models/*)

set(DEMO_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Assignment7.pdf
    ${DEMO_MODELS}
)

set(SRCS main.cpp Object.hpp Vector.cpp Vector.hpp
    Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp
    AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp 
    Intersection.hpp Renderer.cpp Renderer.hpp
)

add_executable(${TARGET_NAME} ${SRCS} ${DEMO_RESOURCES})
target_link_libraries(${TARGET_NAME} threadpool)

set(BUNNY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/bunny/bunny.obj")
set(CORNELL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models/cornellbox")
set(COW_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/cow/cow.obj")

source_group(src FILES ${SRCS})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${DEMO_RESOURCES})

target_compile_definitions(${TARGET_NAME} PRIVATE
    BUNNY_PATH="${BUNNY_PATH}"
    CORNELL_DIR="${CORNELL_DIR}"
    COW_PATH="${COW_PATH}"
    DEFAULT_SPLIT_METHOD=SAH
    CPU_CORE_COUNT=6
)
