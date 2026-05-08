add_subdirectory(optimizer)
add_subdirectory(renderer)
add_subdirectory(sim)

add_library(reefscape INTERFACE)
target_include_directories(reefscape INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
