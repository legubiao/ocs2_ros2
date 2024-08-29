cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ddp)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)

set(dependencies
        ocs2_mpc
        ocs2_qp_solver
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/riccati_equations/ContinuousTimeRiccatiEquations.cpp
        src/riccati_equations/DiscreteTimeRiccatiEquations.cpp
        src/riccati_equations/RiccatiModification.cpp
        src/search_strategy/LevenbergMarquardtStrategy.cpp
        src/search_strategy/LineSearchStrategy.cpp
        src/search_strategy/StrategySettings.cpp
        src/ContinuousTimeLqr.cpp
        src/GaussNewtonDDP.cpp
        src/GaussNewtonDDP_MPC.cpp
        src/HessianCorrection.cpp
        src/ILQR.cpp
        src/SLQ.cpp
        src/DDP_Settings.cpp
        src/DDP_HelperFunctions.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############
install(
        DIRECTORY include/ test/include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############

if (BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(exp0_ddp_test test/Exp0Test.cpp)
    target_link_libraries(exp0_ddp_test ${PROJECT_NAME})
    ament_target_dependencies(exp0_ddp_test ${dependencies})


    ament_add_gtest(exp1_ddp_test test/Exp1Test.cpp)
    target_link_libraries(exp1_ddp_test ${PROJECT_NAME})
    ament_target_dependencies(exp1_ddp_test ${dependencies})


    ament_add_gtest(correctness_test test/CorrectnessTest.cpp)
    target_link_libraries(correctness_test ${PROJECT_NAME})
    ament_target_dependencies(correctness_test ${dependencies})


    ament_add_gtest(riccati_ode_test test/RiccatiTest.cpp)
    target_link_libraries(riccati_ode_test ${PROJECT_NAME})
    ament_target_dependencies(riccati_ode_test ${dependencies})


    ament_add_gtest(circular_kinematics_ddp_test test/CircularKinematicsTest.cpp)
    target_link_libraries(circular_kinematics_ddp_test ${PROJECT_NAME})
    ament_target_dependencies(circular_kinematics_ddp_test ${dependencies})

    ament_add_gtest(hybrid_slq_test test/HybridSlqTest.cpp)
    target_link_libraries(hybrid_slq_test ${PROJECT_NAME})
    ament_target_dependencies(hybrid_slq_test ${dependencies})

    ament_add_gtest(bouncing_mass_test
            test/bouncingmass/BouncingMassTest.cpp
            test/bouncingmass/OverallReference.cpp
            test/bouncingmass/Reference.cpp
    )
    target_link_libraries(bouncing_mass_test ${PROJECT_NAME})
    ament_target_dependencies(bouncing_mass_test ${dependencies})


    ament_add_gtest(testContinuousTimeLqr test/testContinuousTimeLqr.cpp)
    target_link_libraries(testContinuousTimeLqr ${PROJECT_NAME})
    ament_target_dependencies(testContinuousTimeLqr ${dependencies})


    ament_add_gtest(testDdpHelperFunction test/testDdpHelperFunction.cpp)
    target_link_libraries(testDdpHelperFunction ${PROJECT_NAME})
    ament_target_dependencies(testDdpHelperFunction ${dependencies})


    ament_add_gtest(testReachingTask test/testReachingTask.cpp)
    target_link_libraries(testReachingTask ${PROJECT_NAME})
    ament_target_dependencies(testReachingTask ${dependencies})

endif ()

ament_package()