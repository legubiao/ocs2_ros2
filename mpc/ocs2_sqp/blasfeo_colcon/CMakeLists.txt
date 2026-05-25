cmake_minimum_required(VERSION 3.14)
project(blasfeo_colcon)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

# ====================================================================
# Option: prefer pre-installed BLASFEO if its CMake config is available.
# ====================================================================
option(OCS2_USE_SYSTEM_BLASFEO
        "If ON, reuse an existing BLASFEO whose blasfeoConfig.cmake can be located via find_package; otherwise fetch and build from source."
        ON)

set(_blasfeo_from_system FALSE)
if(OCS2_USE_SYSTEM_BLASFEO)
    find_package(blasfeo QUIET CONFIG)
    if(blasfeo_FOUND)
        message(STATUS "blasfeo_colcon: reusing pre-installed BLASFEO (target 'blasfeo' from ${blasfeo_DIR})")

        # Derive BLASFEO_DEVEL_PREFIX from the imported target's include dir so that
        # the configured blasfeo-extras.cmake can expose it to consumers (e.g. hpipm_colcon).
        get_target_property(_blasfeo_inc blasfeo INTERFACE_INCLUDE_DIRECTORIES)
        if(_blasfeo_inc)
            list(GET _blasfeo_inc 0 _blasfeo_first_inc)
            get_filename_component(BLASFEO_DEVEL_PREFIX "${_blasfeo_first_inc}/.." ABSOLUTE)
        else()
            # Fallback: assume the imported target's location prefix.
            get_target_property(_blasfeo_loc blasfeo IMPORTED_LOCATION_RELEASE)
            if(NOT _blasfeo_loc)
                get_target_property(_blasfeo_loc blasfeo IMPORTED_LOCATION)
            endif()
            if(_blasfeo_loc)
                get_filename_component(_blasfeo_libdir "${_blasfeo_loc}" DIRECTORY)
                get_filename_component(BLASFEO_DEVEL_PREFIX "${_blasfeo_libdir}/.." ABSOLUTE)
            else()
                set(BLASFEO_DEVEL_PREFIX "${CMAKE_INSTALL_PREFIX}")
            endif()
        endif()
        set(_blasfeo_from_system TRUE)
    else()
        message(STATUS "blasfeo_colcon: no pre-installed BLASFEO found, building from source")
    endif()
endif()

# ====================================================================
# Fallback: build BLASFEO from source via FetchContent
# ====================================================================
if(NOT _blasfeo_from_system)
    include(FetchContent)

    # When BLASFEO is built from source we install it into our own colcon prefix so that
    # downstream find_package(blasfeo) (triggered via ament_export_dependencies below)
    # picks up the bundled blasfeoConfig.cmake.
    set(BLASFEO_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "Blasfeo install path" FORCE)
    set(BLASFEO_INCLUDE_DIR ${BLASFEO_DEVEL_PREFIX}/include)
    set(BLASFEO_LIB_DIR ${BLASFEO_DEVEL_PREFIX}/lib)
    set(BLASFEO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
    set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

    file(MAKE_DIRECTORY ${BLASFEO_INCLUDE_DIR})
    file(MAKE_DIRECTORY ${BLASFEO_LIB_DIR})

    set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
    set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
    set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")

    # BLASFEO upstream has no release tags; we pin a recent commit on master.
    # Last reviewed: 2026-04-27 (commit author date). Avoids the broken
    # kernel_sgemv_*_libc symbols introduced on master in May 2026.
    # To bump: pick a newer commit from https://github.com/giaf/blasfeo/commits
    # then `rm -rf build/blasfeo_colcon install/blasfeo_colcon` and rebuild.
    FetchContent_Declare(blasfeoDownload
            GIT_REPOSITORY https://github.com/giaf/blasfeo
            GIT_TAG 5cb0a065895479e7dc54c1771fec48548aec0746
            UPDATE_COMMAND ""
            SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
            BINARY_DIR ${BLASFEO_BUILD_DIR}
    )
    FetchContent_MakeAvailable(blasfeoDownload)

    # Mirror the headers into the standard ament include layout.
    file(GLOB_RECURSE HEADERS "${BLASFEO_DOWNLOAD_DIR}/include/*.h")
    foreach(HEADER_FILE ${HEADERS})
        file(COPY ${HEADER_FILE} DESTINATION ${BLASFEO_INCLUDE_DIR})
    endforeach()

    set_target_properties(blasfeo PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BLASFEO_LIB_DIR})

    ament_export_include_directories(include)
endif()

# Re-export the blasfeo target to downstream packages via find_package(blasfeo_colcon).
ament_export_dependencies(blasfeo)

ament_package(CONFIG_EXTRAS "cmake/blasfeo-extras.cmake.in")
