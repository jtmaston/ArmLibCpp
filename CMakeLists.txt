cmake_minimum_required(VERSION 3.22)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(5DOF VERSION 0.1 LANGUAGES CXX)

add_library(5DOF STATIC
    ${PROJECT_SOURCE_DIR}/src/Arm_lib.cpp
)
set_target_properties(5DOF PROPERTIES
    PUBLIC_HEADER "${PROJECT_SOURCE_DIR}/include/Arm_lib.hpp"
    ARCHIVE_OUTPUT_DIRECTORY "lib"
    LIBRARY_OUTPUT_DIRECTORY "lib"
)
target_include_directories(5DOF PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(5DOF
    PUBLIC
    cxx_attributes
    cxx_defaulted_functions
    cxx_deleted_functions
    cxx_final
)
