cmake_minimum_required(VERSION 3.14)

project(
    thing
    VERSION 0.1.0
    LANGUAGES C    
)

include_directories(include)

file(GLOB SOURCES "src/*.c")

add_executable(thing ${SOURCES})