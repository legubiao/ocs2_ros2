cmake_minimum_required(VERSION 3.14)
project(ocs2_ros_interfaces)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find ament_cmake macros and libraries
set(dependencies
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
foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()

###########
## Build ##
###########

## Specify additional locations of header files
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
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(test_custom_callback_queue test/test_custom_callback_queue.cpp)
target_link_libraries(test_custom_callback_queue ${PROJECT_NAME})
target_compile_options(test_custom_callback_queue PRIVATE ${OCS2_CXX_FLAGS})

# multiplot remap node
add_executable(multiplot_remap src/multiplot/MultiplotRemap.cpp)
target_link_libraries(multiplot_remap ${PROJECT_NAME})
target_compile_options(multiplot_remap PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

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

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()