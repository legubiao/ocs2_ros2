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

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

add_library(${PROJECT_NAME}
        src/MobileManipulatorMpcNode.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# MPC node
add_executable(mobile_manipulator_mpc_node src/MobileManipulatorMpcNode.cpp)
target_link_libraries(mobile_manipulator_mpc_node ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_mpc_node ${dependencies})
target_compile_options(mobile_manipulator_mpc_node PUBLIC ${FLAGS})

# DistanceVisualization node
add_executable(mobile_manipulator_distance_visualization src/MobileManipulatorDistanceVisualization.cpp)
target_link_libraries(mobile_manipulator_distance_visualization ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_distance_visualization ${dependencies})
target_compile_options(mobile_manipulator_distance_visualization PUBLIC ${FLAGS})

# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
        src/MobileManipulatorDummyMRT.cpp
        src/MobileManipulatorDummyVisualization.cpp
)
target_link_libraries(mobile_manipulator_dummy_mrt_node ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_dummy_mrt_node ${dependencies})
target_compile_options(mobile_manipulator_dummy_mrt_node PUBLIC ${FLAGS})

# Target node
add_executable(mobile_manipulator_target src/MobileManipulatorTarget.cpp)
target_link_libraries(mobile_manipulator_target ${PROJECT_NAME})
ament_target_dependencies(mobile_manipulator_target ${dependencies})
target_compile_options(mobile_manipulator_target PUBLIC ${FLAGS})

#############
## Install ##
#############

install(DIRECTORY include DESTINATION include/${PROJECT_NAME})
install(
        TARGETS
        mobile_manipulator_mpc_node
        mobile_manipulator_distance_visualization
        mobile_manipulator_dummy_mrt_node
        mobile_manipulator_target
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()