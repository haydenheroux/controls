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
    # Use FetchContent to provide Eigen 5.0.1 instead of relying on a system package.
    # Eigen is header-only; FetchContent will download the sources and make the headers
    # available to the build. If Eigen's build does not create the `Eigen3::Eigen`
    # imported target, create a simple INTERFACE IMPORTED target pointing at the
    # downloaded source include directory so other targets can link to `Eigen3::Eigen`.
    FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 5.0.1
            GIT_SHALLOW TRUE
        )

    FetchContent_MakeAvailable(eigen)

    if (NOT TARGET Eigen3::Eigen)
      add_library(Eigen3::Eigen INTERFACE IMPORTED)
      # The FetchContent import creates variables named <name>_SOURCE_DIR / _BINARY_DIR.
      # For the declared name "eigen" these are `eigen_SOURCE_DIR` and `eigen_BINARY_DIR`.
      # Use the source dir as the include directory for Eigen headers.
      if (DEFINED eigen_SOURCE_DIR)
        set_target_properties(Eigen3::Eigen PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}"
                )
      else()
        message(WARNING "Eigen was fetched but eigen_SOURCE_DIR is not defined; other targets may fail to include Eigen headers.")
      endif()
    endif()
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
