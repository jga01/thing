cmake_minimum_required(VERSION 3.14)

project(
    thing
    VERSION 0.1.0
    LANGUAGES C    
)

file(GLOB SOURCES "src/*.c")
add_executable(thing ${SOURCES})

set(SOKOL_HEADERS "3rdparty/sokol")
set(SOKOLGP_HEADERS "3rdparty/sokol_gp")
set(HANDMADEMATH_HEADERS "3rdparty/HandmadeMath")
set(NUKLEAR_HEADERS "3rdparty/Nuklear")

target_include_directories(thing PRIVATE include)
target_include_directories(
    thing 
    PRIVATE 
    ${SOKOL_HEADERS} 
    ${SOKOLGP_HEADERS} 
    ${HANDMADEMATH_HEADERS} 
    ${NUKLEAR_HEADERS}
    )

add_subdirectory(3rdparty/flecs)

target_link_libraries(thing PRIVATE flecs)
