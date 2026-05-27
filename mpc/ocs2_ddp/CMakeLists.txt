cmake_minimum_required(VERSION 3.14)
project(ocs2_ddp)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)

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
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_mpc_TARGETS}
        ${ocs2_qp_solver_TARGETS}
)

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

ament_export_dependencies(ocs2_mpc ocs2_qp_solver)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(exp0_ddp_test test/Exp0Test.cpp)
    target_link_libraries(exp0_ddp_test ${PROJECT_NAME})

    ament_add_gtest(exp1_ddp_test test/Exp1Test.cpp)
    target_link_libraries(exp1_ddp_test ${PROJECT_NAME})

    ament_add_gtest(correctness_test test/CorrectnessTest.cpp)
    target_link_libraries(correctness_test ${PROJECT_NAME})

    ament_add_gtest(riccati_ode_test test/RiccatiTest.cpp)
    target_link_libraries(riccati_ode_test ${PROJECT_NAME})

    ament_add_gtest(circular_kinematics_ddp_test test/CircularKinematicsTest.cpp)
    target_link_libraries(circular_kinematics_ddp_test ${PROJECT_NAME})

    ament_add_gtest(hybrid_slq_test test/HybridSlqTest.cpp)
    target_link_libraries(hybrid_slq_test ${PROJECT_NAME})

    ament_add_gtest(bouncing_mass_test
            test/bouncingmass/BouncingMassTest.cpp
            test/bouncingmass/OverallReference.cpp
            test/bouncingmass/Reference.cpp
    )
    target_link_libraries(bouncing_mass_test ${PROJECT_NAME})

    ament_add_gtest(testContinuousTimeLqr test/testContinuousTimeLqr.cpp)
    target_link_libraries(testContinuousTimeLqr ${PROJECT_NAME})

    ament_add_gtest(testDdpHelperFunction test/testDdpHelperFunction.cpp)
    target_link_libraries(testDdpHelperFunction ${PROJECT_NAME})

    ament_add_gtest(testReachingTask test/testReachingTask.cpp)
    target_link_libraries(testReachingTask ${PROJECT_NAME})
endif()

ament_package()
