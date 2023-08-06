cmake_minimum_required(VERSION 3.14)

project(
    thing
    VERSION 0.1.0
    LANGUAGES C    
)

file(GLOB SOURCES "src/*.c")
add_executable(thing ${SOURCES})

set(SOKOL_HEADERS "include/sokol")
set(HANDMADEMATH_HEADERS "include/HandmadeMath")

target_include_directories(thing PRIVATE include)
target_include_directories(thing PRIVATE ${SOKOL_HEADERS} ${HANDMADEMATH_HEADERS})

add_subdirectory(include/flecs)

target_link_libraries(thing PRIVATE flecs)
