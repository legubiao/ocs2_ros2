cmake_minimum_required(VERSION 3.14)
project(ocs2_legged_robot_raisim)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(PREPEND CMAKE_PREFIX_PATH "$ENV{HOME}/raisimLib/raisim/linux")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_raisim_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

add_library(${PROJECT_NAME}
        src/RaiSimConversions.cpp
        src/RaiSimVisualizer.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_legged_robot_ros_TARGETS}
        ${ocs2_raisim_core_TARGETS}
        ${grid_map_msgs_TARGETS}
        ament_index_cpp::ament_index_cpp
)

add_executable(legged_robot_raisim_dummy src/RaiSimDummy.cpp)
target_link_libraries(legged_robot_raisim_dummy ${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        legged_robot_raisim_dummy
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ocs2_legged_robot_ros ocs2_raisim_core grid_map_msgs ament_index_cpp)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(${PROJECT_NAME}_test
            test/testLeggedRobotRaisimConversions.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
