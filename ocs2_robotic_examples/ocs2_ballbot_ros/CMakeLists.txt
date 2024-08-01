cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ballbot_ros)

set(dependencies
        rclcpp
        tf2
        urdf
        kdl_parser
        robot_state_publisher
        visualization_msgs
        ocs2_core
        ocs2_ddp
        ocs2_mpc
        ocs2_sqp
        ocs2_slp
        ocs2_ros_interfaces
        ocs2_robotic_tools
        ocs2_ballbot
        ocs2_robotic_assets
        Boost
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(urdf REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_slp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ocs2_ballbot REQUIRED)
find_package(rclcpp REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

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
)
add_library(${PROJECT_NAME} src/BallbotDummyVisualization.cpp)
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(ballbot_ddp src/BallbotDdpMpcNode.cpp)
ament_target_dependencies(ballbot_ddp
        ${dependencies}
)
target_link_libraries(ballbot_ddp
        ${PROJECT_NAME}
)
target_compile_options(ballbot_ddp PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(ballbot_dummy_test src/DummyBallbotNode.cpp)
ament_target_dependencies(ballbot_dummy_test
        ${dependencies}
)
target_link_libraries(ballbot_dummy_test
        ${PROJECT_NAME}
)
target_compile_options(ballbot_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

# # Target node
add_executable(ballbot_target src/BallbotTargetPoseCommand.cpp)
ament_target_dependencies(ballbot_target
        ${dependencies}
)
target_link_libraries(ballbot_target
        ${PROJECT_NAME}
)
target_compile_options(ballbot_target PRIVATE ${OCS2_CXX_FLAGS})

## Combined MPC MRT example
add_executable(ballbot_mpc_mrt
        src/BallbotMpcMrtNode.cpp
        src/BallbotDummyVisualization.cpp)
ament_target_dependencies(ballbot_mpc_mrt
        ${dependencies}
)
target_link_libraries(ballbot_mpc_mrt
        ${PROJECT_NAME}
)
target_compile_options(ballbot_mpc_mrt PRIVATE ${OCS2_CXX_FLAGS})

## SQP node for ballbot
add_executable(ballbot_sqp src/BallbotSqpMpcNode.cpp)
ament_target_dependencies(ballbot_sqp
        ${dependencies}
)
target_link_libraries(ballbot_sqp
        ${PROJECT_NAME}
)
target_compile_options(ballbot_sqp PRIVATE ${OCS2_CXX_FLAGS})

## SLP node for ballbot
add_executable(ballbot_slp src/BallbotSlpMpcNode.cpp)
ament_target_dependencies(ballbot_slp
        ${dependencies}
)
target_link_libraries(ballbot_slp
        ${PROJECT_NAME}
)
target_compile_options(ballbot_slp PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
        TARGETS
        ballbot_ddp
        ballbot_dummy_test
        ballbot_target
        ballbot_sqp
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_ballbot")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            ballbot_ddp
            ballbot_sqp
            ballbot_dummy_test
            ballbot_target
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)


#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

ament_add_gtest(ballbot_integration_test
        test/ballbotIntegrationTest.cpp
)
ament_target_dependencies(ballbot_integration_test
        ${dependencies}
)
target_link_libraries(ballbot_integration_test
        ${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
