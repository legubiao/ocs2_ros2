cmake_minimum_required(VERSION 3.14)
project(hpipm_colcon)

set(CMAKE_BUILD_TYPE Release)

# Set TARGET based on the processor architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(STATUS "Building for Intel x86_64 architecture")
    set(TARGET "AVX")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Building for ARM64 architecture")
    set(TARGET "GENERIC")
else()
    message(STATUS "Building for unknown architecture")
    set(TARGET "GENERIC")
endif()

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(blasfeo_colcon REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
include(FetchContent)

# Define directories
set(HPIPM_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "HPIPM install path")
set(HPIPM_INCLUDE_DIR ${HPIPM_INSTALL_PREFIX}/include)
set(HPIPM_LIB_DIR ${HPIPM_INSTALL_PREFIX}/lib)
set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})
file(MAKE_DIRECTORY ${HPIPM_LIB_DIR})
file(MAKE_DIRECTORY ${HPIPM_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${HPIPM_BUILD_DIR})

message(STATUS "BLASFEO_PATH: " ${HPIPM_INCLUDE_DIR})

# HPIPM Settings
set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(hpipmDownload
        GIT_REPOSITORY https://github.com/giaf/hpipm
        GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
        UPDATE_COMMAND ""
        SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
        BINARY_DIR ${HPIPM_BUILD_DIR}
        BUILD_COMMAND $(MAKE)  TARGET=${TARGET}
        INSTALL_COMMAND "$(MAKE) install"
)
FetchContent_MakeAvailable(hpipmDownload)

# Copy header to where ament_cmake expects them
file(GLOB_RECURSE HEADERS "${HPIPM_DOWNLOAD_DIR}/include/*.h")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: " ${HEADER_FILE})
    file(COPY ${HEADER_FILE} DESTINATION ${HPIPM_INCLUDE_DIR})
endforeach ()

# Install the library where catkin expects them
set_target_properties(hpipm PROPERTIES LIBRARY_OUTPUT_PATH ${HPIPM_LIB_DIR})

###########
## Build ##
###########
set(dependencies
        blasfeo_colcon
        ocs2_qp_solver
)

# Hpipm interface
add_library(${PROJECT_NAME}
        src/HpipmInterface.cpp
        src/HpipmInterfaceSettings.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${HPIPM_INCLUDE_DIR}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION ib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


ament_export_dependencies(${dependencies} hpipm)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME} test/testHpipmInterface.cpp)
    ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME}
            hpipm
    )

endif ()

ament_package()