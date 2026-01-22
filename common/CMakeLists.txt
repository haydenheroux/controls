project(common)

file(GLOB common_src src/Arm.cc src/Elevator.cc src/nt_pubsub.cc)

add_library(common ${common_src})

target_include_directories(common PUBLIC include)

target_compile_features(common PUBLIC cxx_std_23)
target_link_libraries(common PUBLIC Eigen3::Eigen au raylib ntcore)
