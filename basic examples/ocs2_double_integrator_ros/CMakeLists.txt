cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_double_integrator_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_ros_interfaces
        ocs2_double_integrator
        ocs2_robotic_tools
        ocs2_robotic_assets
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_double_integrator REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/DoubleIntegratorMpcNode.cpp
        src/DummyDoubleIntegratorNode.cpp
        src/DoubleIntegratorDummyVisualization.cpp
        src/DoubleIntegratorTargetPoseCommand.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(double_integrator_mpc src/DoubleIntegratorMpcNode.cpp)
target_link_libraries(double_integrator_mpc ${PROJECT_NAME})

# Dummy node
add_executable(double_integrator_dummy_test
        src/DummyDoubleIntegratorNode.cpp
        src/DoubleIntegratorDummyVisualization.cpp
)
target_link_libraries(double_integrator_dummy_test ${PROJECT_NAME})

# Target node
add_executable(double_integrator_target src/DoubleIntegratorTargetPoseCommand.cpp)
target_link_libraries(double_integrator_target ${PROJECT_NAME})

#############
## Install ##
#############

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

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
