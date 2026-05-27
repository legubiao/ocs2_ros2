cmake_minimum_required(VERSION 3.14)
project(ocs2_double_integrator_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_double_integrator REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

add_library(${PROJECT_NAME}
        src/DoubleIntegratorDummyVisualization.cpp
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
        ${ocs2_ros_interfaces_TARGETS}
        ${ocs2_double_integrator_TARGETS}
        ${ocs2_robotic_assets_TARGETS}
        ${ocs2_robotic_tools_TARGETS}
)

add_executable(double_integrator_mpc src/DoubleIntegratorMpcNode.cpp)
target_link_libraries(double_integrator_mpc ${PROJECT_NAME})

add_executable(double_integrator_dummy_test src/DummyDoubleIntegratorNode.cpp)
target_link_libraries(double_integrator_dummy_test ${PROJECT_NAME})

add_executable(double_integrator_target src/DoubleIntegratorTargetPoseCommand.cpp)
target_link_libraries(double_integrator_target ${PROJECT_NAME})

install(
        TARGETS
        double_integrator_mpc
        double_integrator_dummy_test
        double_integrator_target
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
