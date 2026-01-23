function(setup_dependencies)
    include(FetchContent)

    if (NOT TARGET au)
        fetchcontent_declare(
          Au
          GIT_REPOSITORY https://github.com/aurora-opensource/au
          GIT_TAG "main"
          EXCLUDE_FROM_ALL
        )
        set(AU_ENABLE_TESTING OFF CACHE INTERNAL "Enable AU tests")
        fetchcontent_makeavailable(Au)
    endif()

    if (NOT TARGET Eigen3::Eigen)
        find_package(Eigen3 5.0.1 REQUIRED NO_MODULE)
    endif()

    if (NOT TARGET raylib)
        fetchcontent_declare(
            raylib
            GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
            GIT_TAG "master"
            GIT_PROGRESS TRUE
        )

        fetchcontent_makeavailable(raylib)
    endif()

    if (NOT TARGET ntcore)
        set(WITH_CSCORE OFF CACHE INTERNAL "With CSCore")
        set(WITH_GUI OFF CACHE INTERNAL "With GUI")
        set(WITH_JAVA OFF CACHE INTERNAL "With Java")
        set(WITH_NTCORE ON CACHE INTERNAL "With NTCore")
        set(WITH_SIMULATION_MODULES OFF CACHE INTERNAL "With Simulation Modules")
        set(WITH_TESTS OFF CACHE INTERNAL "With Tests")
        set(WITH_WPILIB OFF CACHE INTERNAL "With WPILib")
        set(WITH_WPIMATH OFF CACHE INTERNAL "With WPIMath")
        set(WITH_PROTOBUF OFF CACHE INTERNAL "With protobuf")
        set(WITH_BENCHMARK OFF CACHE INTERNAL "With benchmark")
        set(WITH_WPIUNITS OFF CACHE INTERNAL "With WPIUnits")
        fetchcontent_declare(
            wpilib
            GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
            GIT_TAG main
        )
        fetchcontent_makeavailable(wpilib)
    endif()

    if (NOT TARGET cppzmq)
        find_package(cppzmq)
    endif()

endfunction()
