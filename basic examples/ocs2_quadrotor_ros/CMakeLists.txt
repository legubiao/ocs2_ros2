cmake_minimum_required(VERSION 3.14)
project(ocs2_quadrotor_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_quadrotor REQUIRED)

add_library(${PROJECT_NAME}
        src/QuadrotorDummyVisualization.cpp
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
        ${rclcpp_TARGETS}
        ${tf2_ros_TARGETS}
        ${ocs2_ros_interfaces_TARGETS}
        ${ocs2_quadrotor_TARGETS}
        ament_index_cpp::ament_index_cpp
)

add_executable(quadrotor_mpc src/QuadrotorMpcNode.cpp)
target_link_libraries(quadrotor_mpc ${PROJECT_NAME})

add_executable(quadrotor_dummy_test
        src/DummyQuadrotorNode.cpp
        src/QuadrotorDummyVisualization.cpp
)
target_link_libraries(quadrotor_dummy_test ${PROJECT_NAME})

add_executable(quadrotor_target src/QuadrotorTargetPoseCommand.cpp)
target_link_libraries(quadrotor_target ${PROJECT_NAME})

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
        quadrotor_mpc
        quadrotor_dummy_test
        quadrotor_target
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rclcpp tf2_ros ocs2_ros_interfaces ocs2_quadrotor ament_index_cpp)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
