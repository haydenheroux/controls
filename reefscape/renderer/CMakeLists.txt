project(renderer)

add_executable(renderer main.cc render.cc render.hh render_units.hh)

target_link_libraries(renderer PRIVATE au common ntcore raylib)

target_compile_features(renderer PRIVATE cxx_std_23)
