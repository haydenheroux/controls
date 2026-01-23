cmake_minimum_required(VERSION 3.15...3.31)

project(
    controls
    VERSION 1.0
    DESCRIPTION "Controls toolbox"
    LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/PreventInsource.cmake)

include(Dependencies.cmake)
setup_dependencies()

add_subdirectory(common)
add_subdirectory(optimizer)
# add_subdirectory(points)
add_subdirectory(reefscape)
