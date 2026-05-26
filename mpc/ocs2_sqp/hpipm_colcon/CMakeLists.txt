cmake_minimum_required(VERSION 3.14)
project(hpipm_colcon)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ====================================================================
# Dependencies
# ====================================================================
find_package(ament_cmake REQUIRED)
find_package(blasfeo_colcon REQUIRED)   # provides blasfeo target + BLASFEO_PATH
find_package(ocs2_qp_solver REQUIRED)   # transitively brings ocs2_core + ocs2_oc

# HPIPM upstream has no release tags; we pin a recent commit on master.
# Last reviewed: 2026-05-22 (HEAD at the time of the bump).
# To bump: pick a newer commit from https://github.com/giaf/hpipm/commits
# then `rm -rf build/hpipm_colcon install/hpipm_colcon` and rebuild.
set(HPIPM_GIT_TAG "ff6f47d0715afef77982cbd5570ee41319c6249e")

# HPIPM has no AUTOMATIC target detection (unlike BLASFEO's X64_AUTOMATIC). Its
# CMake only ships two targets: AVX (Intel x86_64 assembly kernels) and GENERIC
# (portable C). We keep AVX on x86_64 (matches upstream's default) and force
# GENERIC everywhere else so that ARM hosts don't try to assemble AVX kernels.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(HPIPM_HOST_TARGET "AVX")
else()
    set(HPIPM_HOST_TARGET "GENERIC")
endif()
message(STATUS "hpipm_colcon: TARGET=${HPIPM_HOST_TARGET} (CPU=${CMAKE_SYSTEM_PROCESSOR})")

# ====================================================================
# Option: prefer pre-installed HPIPM if its CMake config is available.
# ====================================================================
option(OCS2_USE_SYSTEM_HPIPM
        "If ON, reuse an existing HPIPM whose hpipmConfig.cmake can be located via find_package; otherwise fetch and build from source."
        ON)

set(_hpipm_from_system FALSE)
if(OCS2_USE_SYSTEM_HPIPM)
    find_package(hpipm QUIET CONFIG)
    if(hpipm_FOUND)
        message(STATUS "hpipm_colcon: reusing pre-installed HPIPM (target 'hpipm' from ${hpipm_DIR})")
        set(_hpipm_from_system TRUE)
    else()
        message(STATUS "hpipm_colcon: no pre-installed HPIPM found, building from source")
    endif()
endif()

# ====================================================================
# Fallback: build HPIPM from source via FetchContent
# ====================================================================
if(NOT _hpipm_from_system)
    include(FetchContent)

    set(HPIPM_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX})
    set(HPIPM_INCLUDE_DIR ${HPIPM_DEVEL_PREFIX}/include)
    set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
    set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

    file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})

    # BLASFEO_PATH is exported by blasfeo_colcon's CONFIG_EXTRAS (blasfeo-extras.cmake.in).
    if(NOT BLASFEO_PATH)
        message(FATAL_ERROR "BLASFEO_PATH not set; blasfeo_colcon must export it via blasfeo-extras.cmake")
    endif()
    message(STATUS "hpipm_colcon: BLASFEO_PATH = ${BLASFEO_PATH}")

    # HPIPM build configuration (consumed by hpipm's own CMakeLists.txt).
    set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
    set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
    set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled" FORCE)
    set(TARGET "${HPIPM_HOST_TARGET}" CACHE STRING "HPIPM CPU target" FORCE)

    FetchContent_Declare(hpipmDownload
            GIT_REPOSITORY https://github.com/giaf/hpipm
            GIT_TAG ${HPIPM_GIT_TAG}
            UPDATE_COMMAND ""
            SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
            BINARY_DIR ${HPIPM_BUILD_DIR}
    )
    FetchContent_MakeAvailable(hpipmDownload)

    # Mirror the HPIPM headers into our install include directory so downstream
    # code can find them via the install prefix (hpipm's own install rules already
    # place libhpipm.so under ${CMAKE_INSTALL_PREFIX}/lib).
    file(GLOB_RECURSE HEADERS "${HPIPM_DOWNLOAD_DIR}/include/*.h")
    foreach(HEADER_FILE ${HEADERS})
        file(COPY ${HEADER_FILE} DESTINATION ${HPIPM_INCLUDE_DIR})
    endforeach()
endif()

# ====================================================================
# hpipm_colcon wrapper library
# ====================================================================
add_library(${PROJECT_NAME}
        src/HpipmInterface.cpp
        src/HpipmInterfaceSettings.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_qp_solver_TARGETS}
        hpipm
        blasfeo
)

# ====================================================================
# Install
# ====================================================================
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# ====================================================================
# Export
# ====================================================================
ament_export_dependencies(blasfeo_colcon ocs2_qp_solver hpipm)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# ====================================================================
# Tests
# ====================================================================
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME} test/testHpipmInterface.cpp)
    # Plain signature - ament_add_gtest target uses plain target_link_libraries internally.
    # Transitively gets hpipm, blasfeo, ocs2_qp_solver (+ ocs2_oc, ocs2_core) from ${PROJECT_NAME}.
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_package()
