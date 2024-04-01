cmake_minimum_required(VERSION 3.20)

project("LeadFree" C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})
