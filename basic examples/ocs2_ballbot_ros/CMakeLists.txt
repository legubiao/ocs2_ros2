cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ballbot_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        tf2
        urdf
        kdl_parser
        robot_state_publisher
        ocs2_ros_interfaces
        ocs2_ballbot
        ocs2_robotic_assets
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(urdf REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_ballbot REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/BallbotDummyVisualization.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(ballbot_ddp src/BallbotDdpMpcNode.cpp)
target_link_libraries(ballbot_ddp ${PROJECT_NAME})
target_compile_options(ballbot_ddp PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(ballbot_dummy_test src/DummyBallbotNode.cpp)
target_link_libraries(ballbot_dummy_test ${PROJECT_NAME})
target_compile_options(ballbot_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

# # Target node
add_executable(ballbot_target src/BallbotTargetPoseCommand.cpp)
target_link_libraries(ballbot_target ${PROJECT_NAME})
target_compile_options(ballbot_target PRIVATE ${OCS2_CXX_FLAGS})

## Combined MPC MRT example
add_executable(ballbot_mpc_mrt
        src/BallbotMpcMrtNode.cpp
        src/BallbotDummyVisualization.cpp)
target_link_libraries(ballbot_mpc_mrt ${PROJECT_NAME})
target_compile_options(ballbot_mpc_mrt PRIVATE ${OCS2_CXX_FLAGS})

## SQP node for ballbot
add_executable(ballbot_sqp src/BallbotSqpMpcNode.cpp)
target_link_libraries(ballbot_sqp ${PROJECT_NAME})
target_compile_options(ballbot_sqp PRIVATE ${OCS2_CXX_FLAGS})

## SLP node for ballbot
add_executable(ballbot_slp src/BallbotSlpMpcNode.cpp)
target_link_libraries(ballbot_slp ${PROJECT_NAME})
target_compile_options(ballbot_slp PRIVATE ${OCS2_CXX_FLAGS})

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
        ARCHIVE DESTINATION ib
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

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


#############
## Testing ##
#############

if (BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)
    ament_add_gtest(ballbot_integration_test test/ballbotIntegrationTest.cpp)
    ament_target_dependencies(ballbot_integration_test ${dependencies})
    target_link_libraries(ballbot_integration_test ${PROJECT_NAME})

endif ()

ament_package()
