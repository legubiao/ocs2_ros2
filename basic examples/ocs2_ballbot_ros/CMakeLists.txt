cmake_minimum_required(VERSION 3.14)
project(ocs2_ballbot_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_ballbot REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

add_library(${PROJECT_NAME}
        src/BallbotDummyVisualization.cpp
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
        ${urdf_TARGETS}
        ${kdl_parser_TARGETS}
        ${robot_state_publisher_TARGETS}
        ${ocs2_ros_interfaces_TARGETS}
        ${ocs2_ballbot_TARGETS}
        ${ocs2_robotic_assets_TARGETS}
)

add_executable(ballbot_ddp src/BallbotDdpMpcNode.cpp)
target_link_libraries(ballbot_ddp ${PROJECT_NAME})

add_executable(ballbot_dummy_test src/DummyBallbotNode.cpp)
target_link_libraries(ballbot_dummy_test ${PROJECT_NAME})

add_executable(ballbot_target src/BallbotTargetPoseCommand.cpp)
target_link_libraries(ballbot_target ${PROJECT_NAME})

add_executable(ballbot_mpc_mrt
        src/BallbotMpcMrtNode.cpp
        src/BallbotDummyVisualization.cpp
)
target_link_libraries(ballbot_mpc_mrt ${PROJECT_NAME})

add_executable(ballbot_sqp src/BallbotSqpMpcNode.cpp)
target_link_libraries(ballbot_sqp ${PROJECT_NAME})

add_executable(ballbot_slp src/BallbotSlpMpcNode.cpp)
target_link_libraries(ballbot_slp ${PROJECT_NAME})

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
        ballbot_ddp
        ballbot_dummy_test
        ballbot_target
        ballbot_sqp
        ballbot_slp
        ballbot_mpc_mrt
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
        tf2 urdf kdl_parser robot_state_publisher
        ocs2_ros_interfaces ocs2_ballbot ocs2_robotic_assets
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(ballbot_integration_test test/ballbotIntegrationTest.cpp)
    target_link_libraries(ballbot_integration_test ${PROJECT_NAME})
endif()

ament_package()
