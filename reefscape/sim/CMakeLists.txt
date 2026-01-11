project(sim)

add_executable(sim main.cc)

target_link_libraries(sim PRIVATE Eigen3::Eigen au common)

target_compile_features(sim PRIVATE cxx_std_23)
