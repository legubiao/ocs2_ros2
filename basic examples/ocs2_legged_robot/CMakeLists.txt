cmake_minimum_required(VERSION 3.14)
project(ocs2_legged_robot)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
# ocs2_centroidal_model transitively brings ocs2_pinocchio_interface, whose
# CONFIG_EXTRAS exports OCS2_PINOCCHIO_FLAGS used below.

configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

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

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_ddp_TARGETS}
        ${ocs2_mpc_TARGETS}
        ${ocs2_sqp_TARGETS}
        ${ocs2_ipm_TARGETS}
        ${ocs2_centroidal_model_TARGETS}
)

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

ament_export_dependencies(ocs2_ddp ocs2_mpc ocs2_sqp ocs2_ipm ocs2_centroidal_model)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    find_package(ocs2_robotic_assets REQUIRED)

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
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${ocs2_robotic_assets_TARGETS})
endif()

ament_package()
