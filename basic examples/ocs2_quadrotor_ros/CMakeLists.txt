cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadrotor_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        rclcpp
        tf2_ros
        ocs2_ros_interfaces
        ocs2_quadrotor
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_quadrotor REQUIRED)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/QuadrotorDummyVisualization.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# MPC node
add_executable(quadrotor_mpc src/QuadrotorMpcNode.cpp)
target_link_libraries(quadrotor_mpc ${PROJECT_NAME})
ament_target_dependencies(quadrotor_mpc ${dependencies})

# Dummy node
add_executable(quadrotor_dummy_test
        src/DummyQuadrotorNode.cpp
        src/QuadrotorDummyVisualization.cpp
)
target_link_libraries(quadrotor_dummy_test ${PROJECT_NAME})
ament_target_dependencies(quadrotor_dummy_test ${dependencies})

# Target node
add_executable(quadrotor_target src/QuadrotorTargetPoseCommand.cpp)
target_link_libraries(quadrotor_target ${PROJECT_NAME})
ament_target_dependencies(quadrotor_target ${dependencies})

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
        quadrotor_mpc
        quadrotor_dummy_test
        quadrotor_target
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(
        DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()