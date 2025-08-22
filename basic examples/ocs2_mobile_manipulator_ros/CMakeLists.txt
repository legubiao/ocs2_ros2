cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_mobile_manipulator_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        tf2_ros
        urdf
        kdl_parser
        robot_state_publisher
        ocs2_self_collision_visualization
        ocs2_mobile_manipulator
)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/MobileManipulatorMpcNode.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# MPC node
add_executable(mobile_manipulator_mpc_node src/MobileManipulatorMpcNode.cpp)
target_link_libraries(mobile_manipulator_mpc_node ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_mpc_node ${dependencies})

# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
        src/MobileManipulatorDummyMRT.cpp
        src/MobileManipulatorDummyVisualization.cpp
)
target_link_libraries(mobile_manipulator_dummy_mrt_node ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_dummy_mrt_node ${dependencies})

# Target node
add_executable(mobile_manipulator_target src/MobileManipulatorTarget.cpp)
target_link_libraries(mobile_manipulator_target ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_target ${dependencies})

#############
## Install ##
#############

install(DIRECTORY include DESTINATION include/${PROJECT_NAME})
install(
        TARGETS
        mobile_manipulator_mpc_node
        mobile_manipulator_dummy_mrt_node
        mobile_manipulator_target
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()