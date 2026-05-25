cmake_minimum_required(VERSION 3.14)
project(ocs2_ros_interfaces)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ament_dependencies
        rclcpp
        rclcpp_lifecycle
        ocs2_msgs
        ocs2_mpc
        std_msgs
        visualization_msgs
        geometry_msgs
        interactive_markers
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${ament_dependencies})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
        src/command/TargetTrajectoriesRosPublisher.cpp
        src/command/UnifiedTargetTrajectoriesInteractiveMarker.cpp
        src/command/JoystickMarkerWrapper.cpp
        src/command/MarkerAutoPositionWrapper.cpp
        src/command/TargetTrajectoriesKeyboardPublisher.cpp
        src/common/RosMsgConversions.cpp
        src/common/RosMsgHelpers.cpp
        src/mpc/MPC_ROS_Interface.cpp
        src/mrt/LoopshapingDummyObserver.cpp
        src/mrt/MRT_ROS_Dummy_Loop.cpp
        src/mrt/MRT_ROS_Interface.cpp
        src/synchronized_module/RosReferenceManager.cpp
        src/synchronized_module/SolverObserverRosCallbacks.cpp
        src/visualization/VisualizationHelpers.cpp
        src/visualization/VisualizationColors.cpp
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
        ${rclcpp_lifecycle_TARGETS}
        ${ocs2_msgs_TARGETS}
        ${ocs2_mpc_TARGETS}
        ${std_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${interactive_markers_TARGETS}
)

add_executable(test_custom_callback_queue test/test_custom_callback_queue.cpp)
target_link_libraries(test_custom_callback_queue ${PROJECT_NAME})

add_executable(multiplot_remap src/multiplot/MultiplotRemap.cpp)
target_link_libraries(multiplot_remap ${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        DIRECTORY launch multiplot
        DESTINATION share/${PROJECT_NAME}
)

install(
        TARGETS multiplot_remap
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${ament_dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
