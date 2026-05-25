cmake_minimum_required(VERSION 3.14)
project(ocs2_quadruped_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_anymal_commands REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(segmented_planes_terrain_model REQUIRED)

add_library(${PROJECT_NAME}
        src/QuadrupedDummyNode.cpp
        src/QuadrupedInterface.cpp
        src/QuadrupedLogger.cpp
        src/QuadrupedMpc.cpp
        src/QuadrupedMpcNode.cpp
        src/QuadrupedPointfootInterface.cpp
        src/QuadrupedTfPublisher.cpp
        src/QuadrupedVisualizer.cpp
        src/SwingPlanningVisualizer.cpp
        src/TerrainPlaneVisualizer.cpp
        src/TerrainReceiver.cpp
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
        ${tf2_TARGETS}
        ${kdl_parser_TARGETS}
        ${robot_state_publisher_TARGETS}
        ${ocs2_anymal_commands_TARGETS}
        ${ocs2_ddp_TARGETS}
        ${ocs2_sqp_TARGETS}
        ${segmented_planes_terrain_model_TARGETS}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
        tf2 kdl_parser robot_state_publisher
        ocs2_anymal_commands ocs2_ddp ocs2_sqp segmented_planes_terrain_model
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
