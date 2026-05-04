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
    FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 5.0.1
            GIT_SHALLOW TRUE
        )

    FetchContent_MakeAvailable(eigen)

    if (NOT TARGET Eigen3::Eigen)
      add_library(Eigen3::Eigen INTERFACE IMPORTED)
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

  if (NOT TARGET cppzmq)
    find_package(cppzmq)
  endif()

endfunction()
