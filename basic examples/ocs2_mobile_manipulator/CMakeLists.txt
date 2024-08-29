cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_mobile_manipulator)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_ddp
        ocs2_robotic_assets
        ocs2_self_collision
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_self_collision REQUIRED)

###########
## Build ##
###########

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

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
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

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
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest)

    ament_add_gtest(${PROJECT_NAME}_test
            test/testSelfCollision.cpp
            test/testEndEffectorConstraint.cpp
            test/testDummyMobileManipulator.cpp
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE
            test/include
            ${PROJECT_BINARY_DIR}/include
    )
    ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})

endif ()

ament_package()