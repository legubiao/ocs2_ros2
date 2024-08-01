cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_mobile_manipulator)


set(dependencies
        ocs2_core
        ocs2_ddp
        ocs2_mpc
        ocs2_robotic_tools
        ocs2_robotic_assets
        ocs2_pinocchio_interface
        ocs2_self_collision
        Boost
        pinocchio
        hpp-fcl
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_self_collision REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

# Add directories for all targets
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
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

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

# Helper macro for adding target applications
macro(add_ocs2_test APP_NAME APP_SRC)
    ament_add_gtest(${APP_NAME}
            ${APP_SRC}
    )
    target_include_directories(${APP_NAME} PRIVATE
            ${PROJECT_BINARY_DIR}/include
    )
    ament_target_dependencies(${APP_NAME}
            ${dependencies}
    )
    target_link_libraries(${APP_NAME}
            ${PROJECT_NAME}
    )
endmacro()

add_ocs2_test(SelfCollisionTest test/testSelfCollision.cpp)
add_ocs2_test(EndEffectorConstraintTest test/testEndEffectorConstraint.cpp)
add_ocs2_test(DummyMobileManipulatorTest test/testDummyMobileManipulator.cpp)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()