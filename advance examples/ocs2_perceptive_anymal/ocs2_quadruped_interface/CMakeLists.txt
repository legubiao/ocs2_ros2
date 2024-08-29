cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadruped_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## ament_cmake Dependencies
set(dependencies
        tf2
        kdl_parser
        robot_state_publisher
        ocs2_anymal_commands
        ocs2_ddp
        ocs2_sqp
        segmented_planes_terrain_model
)

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(ocs2_anymal_commands REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(segmented_planes_terrain_model REQUIRED)

###########
## Build ##
###########
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
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

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

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()