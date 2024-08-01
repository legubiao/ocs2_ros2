cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_mobile_manipulator_ros)


set(dependencies
        rclcpp
        tf2_ros
        urdf
        kdl_parser
        robot_state_publisher
        visualization_msgs
        geometry_msgs
        ocs2_ros_interfaces
        ocs2_core
        ocs2_ddp
        ocs2_mpc
        ocs2_robotic_tools
        ocs2_robotic_assets
        ocs2_pinocchio_interface
        ocs2_self_collision
        ocs2_self_collision_visualization
        ocs2_mobile_manipulator
        Boost
        pinocchio
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)

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

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

# MPC node
add_executable(mobile_manipulator_mpc_node
        src/MobileManipulatorMpcNode.cpp
)
ament_target_dependencies(mobile_manipulator_mpc_node
        ${dependencies}
)
target_compile_options(mobile_manipulator_mpc_node PUBLIC ${FLAGS})

# DistanceVisualization node
add_executable(mobile_manipulator_distance_visualization
        src/MobileManipulatorDistanceVisualization.cpp
)
ament_target_dependencies(mobile_manipulator_distance_visualization
        ${dependencies}
)
target_compile_options(mobile_manipulator_distance_visualization PUBLIC ${FLAGS})

# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
        src/MobileManipulatorDummyMRT.cpp
        src/MobileManipulatorDummyVisualization.cpp
)
ament_target_dependencies(mobile_manipulator_dummy_mrt_node
        ${dependencies}
)
target_compile_options(mobile_manipulator_dummy_mrt_node PUBLIC ${FLAGS})

# Target node
add_executable(mobile_manipulator_target
        src/MobileManipulatorTarget.cpp
)
ament_target_dependencies(mobile_manipulator_target
        ${dependencies}
)
target_compile_options(mobile_manipulator_target PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
            TARGETS mobile_manipulator_mpc_node mobile_manipulator_dummy_mrt_node
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
        TARGETS
        mobile_manipulator_mpc_node
        mobile_manipulator_distance_visualization
        mobile_manipulator_dummy_mrt_node
        mobile_manipulator_target
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()