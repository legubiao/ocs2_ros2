cmake_minimum_required(VERSION 3.14)
project(ocs2_legged_robot_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

add_library(${PROJECT_NAME}
        src/gait/GaitKeyboardPublisher.cpp
        src/gait/GaitReceiver.cpp
        src/visualization/LeggedRobotVisualizer.cpp
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
        ${tf2_ros_TARGETS}
        ${robot_state_publisher_TARGETS}
        ${ocs2_legged_robot_TARGETS}
        ${ocs2_ros_interfaces_TARGETS}
)

add_executable(legged_robot_ddp_mpc src/LeggedRobotDdpMpcNode.cpp)
target_link_libraries(legged_robot_ddp_mpc ${PROJECT_NAME})

add_executable(legged_robot_sqp_mpc src/LeggedRobotSqpMpcNode.cpp)
target_link_libraries(legged_robot_sqp_mpc ${PROJECT_NAME})

add_executable(legged_robot_ipm_mpc src/LeggedRobotIpmMpcNode.cpp)
target_link_libraries(legged_robot_ipm_mpc ${PROJECT_NAME})

add_executable(legged_robot_dummy src/LeggedRobotDummyNode.cpp)
target_link_libraries(legged_robot_dummy ${PROJECT_NAME})

add_executable(legged_robot_target src/LeggedRobotPoseCommandNode.cpp)
target_link_libraries(legged_robot_target ${PROJECT_NAME})

add_executable(legged_robot_gait_command src/LeggedRobotGaitCommandNode.cpp)
target_link_libraries(legged_robot_gait_command ${PROJECT_NAME})

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

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(tf2_ros robot_state_publisher ocs2_legged_robot ocs2_ros_interfaces)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
