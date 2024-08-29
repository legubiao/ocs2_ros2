cmake_minimum_required(VERSION 3.14)
project(blasfeo_colcon)

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
include(FetchContent)

# Define directories
set(BLASFEO_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "Blasfeo install path")
set(BLASFEO_INCLUDE_DIR ${BLASFEO_INSTALL_PREFIX}/include)
set(BLASFEO_LIB_DIR ${BLASFEO_INSTALL_PREFIX}/lib)
set(BLASFEO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${BLASFEO_INCLUDE_DIR})
file(MAKE_DIRECTORY ${BLASFEO_LIB_DIR})
file(MAKE_DIRECTORY ${BLASFEO_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${BLASFEO_BUILD_DIR})

# BLASFEO Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(blasfeoDownload
        GIT_REPOSITORY https://github.com/giaf/blasfeo
        GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
        UPDATE_COMMAND ""
        SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
        BINARY_DIR ${BLASFEO_BUILD_DIR}
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND "$(MAKE) install"
)
FetchContent_MakeAvailable(blasfeoDownload)

# Copy header to where ament_cmake expects them
file(GLOB_RECURSE HEADERS "${BLASFEO_DOWNLOAD_DIR}/include/*.h")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: ${HEADER_FILE}")
    file(COPY ${HEADER_FILE} DESTINATION ${BLASFEO_INCLUDE_DIR})
endforeach ()

# Install the library where ament_cmake expects them
set_target_properties(blasfeo PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BLASFEO_LIB_DIR})

# Propagate dependencies
ament_export_include_directories(include)
ament_export_dependencies(blasfeo)

ament_package(CONFIG_EXTRAS "cmake/blasfeo-extras.cmake.in")