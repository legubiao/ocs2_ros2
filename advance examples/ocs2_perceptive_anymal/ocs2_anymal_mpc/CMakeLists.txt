cmake_minimum_required(VERSION 3.14)
project(ocs2_anymal_mpc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_anymal_models REQUIRED)
find_package(ocs2_quadruped_interface REQUIRED)

add_library(${PROJECT_NAME}
        src/AnymalInterface.cpp
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
        ${ocs2_anymal_models_TARGETS}
        ${ocs2_quadruped_interface_TARGETS}
)

add_executable(${PROJECT_NAME}_mpc_node src/AnymalMpcNode.cpp)
target_link_libraries(${PROJECT_NAME}_mpc_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_dummy_mrt_node src/AnymalDummyMRT.cpp)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node ${PROJECT_NAME})

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
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

install(
        TARGETS
        ${PROJECT_NAME}_mpc_node
        ${PROJECT_NAME}_dummy_mrt_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ocs2_anymal_models ocs2_quadruped_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(${PROJECT_NAME}_test test/testProblemFormulation.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
