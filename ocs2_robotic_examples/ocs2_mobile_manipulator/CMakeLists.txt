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
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
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
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
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
ament_target_dependencies(${PROJECT_NAME}_test
        ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()