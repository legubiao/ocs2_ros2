cmake_minimum_required(VERSION 3.14)
project(ocs2_anymal_commands)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_switched_model_interface REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)

add_library(${PROJECT_NAME}
        src/LoadMotions.cpp
        src/ModeSequenceKeyboard.cpp
        src/MotionCommandController.cpp
        src/MotionCommandDummy.cpp
        src/MotionCommandInterface.cpp
        src/PoseCommandToCostDesiredRos.cpp
        src/ReferenceExtrapolation.cpp
        src/TerrainAdaptation.cpp
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
        ${ocs2_switched_model_interface_TARGETS}
        ${grid_map_filters_rsl_TARGETS}
)

add_executable(target_command_node src/AnymalPoseCommandNode.cpp)
target_link_libraries(target_command_node ${PROJECT_NAME})

add_executable(gait_command_node src/AnymalGaitNode.cpp)
target_link_libraries(gait_command_node ${PROJECT_NAME})

add_executable(motion_command_node src/AnymalMotionCommandNode.cpp)
target_link_libraries(motion_command_node ${PROJECT_NAME})

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

install(TARGETS
        target_command_node
        gait_command_node
        motion_command_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ocs2_switched_model_interface grid_map_filters_rsl)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(test_${PROJECT_NAME}
            test/testLoadMotions.cpp
            test/testReferenceExtrapolation.cpp
            test/testTerrainAdaptation.cpp
    )
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} -lstdc++fs)
endif()

ament_package()
