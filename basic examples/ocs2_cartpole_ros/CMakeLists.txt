cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_cartpole_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        rclcpp
        ocs2_ros_interfaces
        ocs2_cartpole
        ocs2_robotic_assets
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_cartpole REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/CartpoleMpcNode.cpp
        src/DummyCartpoleNode.cpp
        src/CartpoleDummyVisualization.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(cartpole_mpc src/CartpoleMpcNode.cpp)
target_link_libraries(cartpole_mpc ${PROJECT_NAME})

# Dummy node
add_executable(cartpole_dummy_test
        src/DummyCartpoleNode.cpp
        src/CartpoleDummyVisualization.cpp
)
target_link_libraries(cartpole_dummy_test ${PROJECT_NAME})

#############
## Install ##
#############

install(
        TARGETS
        cartpole_mpc
        cartpole_dummy_test
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
