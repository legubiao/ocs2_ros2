cmake_minimum_required(VERSION 3.14)
project(ocs2_anymal_loopshaping_mpc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_anymal_mpc REQUIRED)
find_package(ocs2_quadruped_loopshaping_interface REQUIRED)

add_library(${PROJECT_NAME}
        src/AnymalLoopshapingInterface.cpp
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
        ${ocs2_anymal_mpc_TARGETS}
        ${ocs2_quadruped_loopshaping_interface_TARGETS}
)

add_executable(${PROJECT_NAME}_mpc_node src/AnymalLoopshapingMpcNode.cpp)
target_link_libraries(${PROJECT_NAME}_mpc_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_dummy_mrt_node src/AnymalLoopshapingDummyMrt.cpp)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_perceptive_demo src/PerceptiveMpcDemo.cpp)
target_link_libraries(${PROJECT_NAME}_perceptive_demo ${PROJECT_NAME})

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
        ${PROJECT_NAME}_mpc_node
        ${PROJECT_NAME}_dummy_mrt_node
        ${PROJECT_NAME}_perceptive_demo
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ocs2_anymal_mpc ocs2_quadruped_loopshaping_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(${PROJECT_NAME}_test
            test/testProblemFormulation.cpp
            test/testMotionTracking.cpp
            test/testSensitivity.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
