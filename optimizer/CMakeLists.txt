project(optimizer)

add_executable(optimizer main.cc)

target_link_libraries(optimizer PRIVATE Eigen3::Eigen au common)

target_compile_features(optimizer PRIVATE cxx_std_23)
