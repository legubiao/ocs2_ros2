cmake_minimum_required(VERSION 3.14)
project(ocs2_mobile_manipulator)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build the testing tree." OFF)

find_package(ament_cmake REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_self_collision REQUIRED)
# ocs2_self_collision transitively brings ocs2_pinocchio_interface, whose
# CONFIG_EXTRAS exports OCS2_PINOCCHIO_FLAGS used below.

configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

add_library(${PROJECT_NAME}
        src/FactoryFunctions.cpp
        src/MobileManipulatorPreComputation.cpp
        src/MobileManipulatorPinocchioMapping.cpp
        src/constraint/EndEffectorConstraint.cpp
        src/constraint/BodyRelativeConstraint.cpp
        src/constraint/EnvironmentCollisionConstraint.cpp
        src/collision/EnvironmentGeometryInterface.cpp
        src/dynamics/WheelBasedMobileManipulatorDynamics.cpp
        src/dynamics/DefaultManipulatorDynamics.cpp
        src/dynamics/FloatingArmManipulatorDynamics.cpp
        src/dynamics/FullyActuatedFloatingArmManipulatorDynamics.cpp
        src/MobileManipulatorInterface.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_ddp_TARGETS}
        ${ocs2_sqp_TARGETS}
        ${ocs2_self_collision_TARGETS}
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        FILES "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h"
        DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ocs2_ddp ocs2_sqp ocs2_self_collision)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)
    find_package(ocs2_robotic_assets REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test
            test/testSelfCollision.cpp
            test/testEndEffectorConstraint.cpp
            test/testDummyMobileManipulator.cpp
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE test/include)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${ocs2_robotic_assets_TARGETS})
endif()

ament_package()
