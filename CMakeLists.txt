cmake_minimum_required(VERSION 3.18)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(5DOF VERSION 0.1 LANGUAGES CXX)
SET(CMAKE_CXX_STANDARD 20)


add_library(5DOF STATIC
    ${PROJECT_SOURCE_DIR}/src/Arm_lib.cpp
)
set_target_properties(5DOF PROPERTIES
    PUBLIC_HEADER "${PROJECT_SOURCE_DIR}/include/Arm_lib.hpp"
)
target_include_directories(5DOF PUBLIC ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/lib/serial/include)

target_link_libraries(5DOF PUBLIC serial)
