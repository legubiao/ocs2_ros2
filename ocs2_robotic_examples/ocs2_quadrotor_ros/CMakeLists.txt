cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadrotor_ros)


set(dependencies
        rclcpp
        tf2_ros
        ocs2_mpc
        ocs2_ros_interfaces
        ocs2_robotic_tools
        ocs2_robotic_assets
        ocs2_quadrotor
        Boost
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_quadrotor REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        dependencies
)

# MPC node
add_executable(quadrotor_mpc
        src/QuadrotorMpcNode.cpp
)
ament_target_dependencies(quadrotor_mpc
        ${dependencies}
)

# Dummy node
add_executable(quadrotor_dummy_test
        src/DummyQuadrotorNode.cpp
        src/QuadrotorDummyVisualization.cpp
)
ament_target_dependencies(quadrotor_dummy_test
        ${dependencies}
)

# Target node
add_executable(quadrotor_target
        src/QuadrotorTargetPoseCommand.cpp
)
ament_target_dependencies(quadrotor_target
        ${dependencies}
)
target_compile_options(quadrotor_target PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_quadrotor")
    add_clang_tooling(
            TARGETS
            quadrotor_mpc
            quadrotor_dummy_test
            quadrotor_target
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
        quadrotor_mpc
        quadrotor_dummy_test
        quadrotor_target
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()