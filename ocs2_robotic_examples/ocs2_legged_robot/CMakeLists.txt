cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_legged_robot)


set(dependencies
        ocs2_core
        ocs2_oc
        ocs2_ddp
        ocs2_mpc
        ocs2_sqp
        ocs2_ipm
        ocs2_robotic_tools
        ocs2_pinocchio_interface
        ocs2_centroidal_model
        ocs2_robotic_assets
        Boost
        pinocchio
        urdf
        urdfdom
        hpp-fcl
)

find_package(hpp-fcl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
        include
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# Legged robot interface library
add_library(${PROJECT_NAME}
        src/common/ModelSettings.cpp
        src/dynamics/LeggedRobotDynamicsAD.cpp
        src/constraint/EndEffectorLinearConstraint.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/constraint/ZeroVelocityConstraintCppAd.cpp
        src/initialization/LeggedRobotInitializer.cpp
        src/reference_manager/SwitchedModelReferenceManager.cpp
        src/foot_planner/CubicSpline.cpp
        src/foot_planner/SplineCpg.cpp
        src/foot_planner/SwingTrajectoryPlanner.cpp
        src/gait/Gait.cpp
        src/gait/GaitSchedule.cpp
        src/gait/LegLogic.cpp
        src/gait/ModeSequenceTemplate.cpp
        src/LeggedRobotInterface.cpp
        src/LeggedRobotPreComputation.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_legged_robot")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
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
        test/AnymalFactoryFunctions.cpp
        test/constraint/testEndEffectorLinearConstraint.cpp
        test/constraint/testFrictionConeConstraint.cpp
        test/constraint/testZeroForceConstraint.cpp
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