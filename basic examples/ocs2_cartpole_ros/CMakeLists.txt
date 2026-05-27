cmake_minimum_required(VERSION 3.14)
project(ocs2_cartpole_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_cartpole REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

add_library(${PROJECT_NAME}
        src/CartpoleDummyVisualization.cpp
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
        ${ocs2_ros_interfaces_TARGETS}
        ${ocs2_cartpole_TARGETS}
        ${ocs2_robotic_assets_TARGETS}
)

add_executable(cartpole_mpc src/CartpoleMpcNode.cpp)
target_link_libraries(cartpole_mpc ${PROJECT_NAME})

add_executable(cartpole_dummy_test src/DummyCartpoleNode.cpp)
target_link_libraries(cartpole_dummy_test ${PROJECT_NAME})

install(
        TARGETS
        cartpole_mpc
        cartpole_dummy_test
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
