cmake_minimum_required(VERSION 3.14)
project(ocs2_mobile_manipulator_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)

add_library(${PROJECT_NAME}
        src/MobileManipulatorMpcNode.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${tf2_ros_TARGETS}
        ${urdf_TARGETS}
        ${kdl_parser_TARGETS}
        ${robot_state_publisher_TARGETS}
        ${ocs2_self_collision_visualization_TARGETS}
        ${ocs2_mobile_manipulator_TARGETS}
)

add_executable(mobile_manipulator_mpc_node src/MobileManipulatorMpcNode.cpp)
target_link_libraries(mobile_manipulator_mpc_node ${PROJECT_NAME})

add_executable(mobile_manipulator_sqp_mpc_node src/MobileManipulatorSqpMpcNode.cpp)
target_link_libraries(mobile_manipulator_sqp_mpc_node ${PROJECT_NAME})

add_executable(mobile_manipulator_dummy_mrt_node
        src/MobileManipulatorDummyMRT.cpp
        src/MobileManipulatorDummyVisualization.cpp
        src/EnvironmentCollisionVisualization.cpp
)
target_link_libraries(mobile_manipulator_dummy_mrt_node ${PROJECT_NAME})

add_executable(mobile_manipulator_target src/MobileManipulatorTarget.cpp)
target_link_libraries(mobile_manipulator_target ${PROJECT_NAME})

install(DIRECTORY include DESTINATION include/${PROJECT_NAME})
install(
        TARGETS
        mobile_manipulator_mpc_node
        mobile_manipulator_sqp_mpc_node
        mobile_manipulator_dummy_mrt_node
        mobile_manipulator_target
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
