cmake_minimum_required(VERSION 3.14)
project(segmented_planes_terrain_model)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_cmake dependencies
set(dependencies
        convex_plane_decomposition_ros
        ocs2_switched_model_interface
        grid_map_sdf
)

find_package(ament_cmake REQUIRED)

find_package(convex_plane_decomposition_ros REQUIRED)
find_package(ocs2_switched_model_interface REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(PCL REQUIRED)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/SegmentedPlanesTerrainModel.cpp
        src/SegmentedPlanesTerrainModelRos.cpp
        src/SegmentedPlanesTerrainVisualization.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PCL_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC -DCGAL_HAS_THREADS)

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
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()