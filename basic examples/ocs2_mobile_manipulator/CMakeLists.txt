cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_mobile_manipulator)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set BUILD_TESTING to OFF by default
option(BUILD_TESTING "Build the testing tree." OFF)

set(dependencies
        ocs2_ddp
        ocs2_self_collision
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_self_collision REQUIRED)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# mobile maniulator interface library
add_library(${PROJECT_NAME}
        src/FactoryFunctions.cpp
        src/MobileManipulatorPreComputation.cpp
        src/MobileManipulatorPinocchioMapping.cpp
        src/constraint/EndEffectorConstraint.cpp
        src/constraint/BodyRelativeConstraint.cpp
        src/dynamics/WheelBasedMobileManipulatorDynamics.cpp
        src/dynamics/DefaultManipulatorDynamics.cpp
        src/dynamics/FloatingArmManipulatorDynamics.cpp
        src/dynamics/FullyActuatedFloatingArmManipulatorDynamics.cpp
        src/MobileManipulatorInterface.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Install ##
#############

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
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

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)
    find_package(ocs2_robotic_assets REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test
            test/testSelfCollision.cpp
            test/testEndEffectorConstraint.cpp
            test/testDummyMobileManipulator.cpp
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE
            test/include
            ${PROJECT_BINARY_DIR}/include
    )
    ament_target_dependencies(${PROJECT_NAME}_test ${dependencies} ocs2_robotic_assets)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})

endif ()

ament_package()