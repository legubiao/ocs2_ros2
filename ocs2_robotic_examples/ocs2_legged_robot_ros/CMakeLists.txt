cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_legged_robot_ros)


set(dependencies
        rclcpp
        tf2_ros
        urdf
        kdl_parser
        robot_state_publisher
        ocs2_core
        ocs2_oc
        ocs2_ddp
        ocs2_mpc
        ocs2_sqp
        ocs2_ipm
        ocs2_robotic_tools
        ocs2_pinocchio_interface
        ocs2_centroidal_model
        ocs2_robotic_assets
        ocs2_msgs
        ocs2_ros_interfaces
        ocs2_legged_robot
        Boost
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(urdf REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(rclcpp REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

# Main library
add_library(${PROJECT_NAME}
        src/gait/GaitKeyboardPublisher.cpp
        src/gait/GaitReceiver.cpp
        src/visualization/LeggedRobotVisualizer.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# DDP-MPC node for legged robot
add_executable(legged_robot_ddp_mpc src/LeggedRobotDdpMpcNode.cpp)
ament_target_dependencies(legged_robot_ddp_mpc ${dependencies})
target_link_libraries(legged_robot_ddp_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc src/LeggedRobotSqpMpcNode.cpp)
ament_target_dependencies(legged_robot_sqp_mpc ${dependencies})
target_link_libraries(legged_robot_sqp_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_sqp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## IPM-MPC node for legged robot
add_executable(legged_robot_ipm_mpc src/LeggedRobotIpmMpcNode.cpp)
ament_target_dependencies(legged_robot_ipm_mpc ${dependencies})
target_link_libraries(legged_robot_ipm_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_ipm_mpc PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(legged_robot_dummy src/LeggedRobotDummyNode.cpp)
ament_target_dependencies(legged_robot_dummy ${dependencies})
target_link_libraries(legged_robot_dummy ${PROJECT_NAME})
target_compile_options(legged_robot_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(legged_robot_target src/LeggedRobotPoseCommandNode.cpp)
ament_target_dependencies(legged_robot_target ${dependencies})
target_link_libraries(legged_robot_target ${PROJECT_NAME})
target_compile_options(legged_robot_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(legged_robot_gait_command src/LeggedRobotGaitCommandNode.cpp)
ament_target_dependencies(legged_robot_gait_command ${dependencies})
target_link_libraries(legged_robot_gait_command ${PROJECT_NAME})
target_compile_options(legged_robot_gait_command PRIVATE ${OCS2_CXX_FLAGS})

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            legged_robot_ddp_mpc
            legged_robot_sqp_mpc
            legged_robot_ipm_mpc
            legged_robot_dummy
            legged_robot_target
            legged_robot_gait_command
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif(cmake_clang_tools_FOUND)


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
        INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(
        TARGETS
        legged_robot_ddp_mpc
        legged_robot_sqp_mpc
        legged_robot_ipm_mpc
        legged_robot_dummy
        legged_robot_target
        legged_robot_gait_command
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        rviz
        DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()