project(logger)

add_executable(logger main.cc)

target_link_libraries(logger PRIVATE common)

target_compile_features(logger PRIVATE cxx_std_23)
