cmake_minimum_required(VERSION 3.14)
project(segmented_planes_terrain_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(ocs2_switched_model_interface REQUIRED)
find_package(grid_map_sdf REQUIRED)
# grid_map_sdf publicly includes pcl/point_cloud.h but does not propagate PCL
# headers via its imported target. Resolve PCL explicitly so downstream usage
# of grid_map_sdf headers works.
find_package(PCL REQUIRED COMPONENTS common)

add_library(${PROJECT_NAME}
        src/SegmentedPlanesTerrainModel.cpp
        src/SegmentedPlanesTerrainModelRos.cpp
        src/SegmentedPlanesTerrainVisualization.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${grid_map_sdf_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS} CGAL_HAS_THREADS)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${convex_plane_decomposition_ros_TARGETS}
        ${ocs2_switched_model_interface_TARGETS}
        ${grid_map_sdf_LIBRARIES}
        ${PCL_LIBRARIES}
)

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

ament_export_dependencies(convex_plane_decomposition_ros ocs2_switched_model_interface grid_map_sdf PCL)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
