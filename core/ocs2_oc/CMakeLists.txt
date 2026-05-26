cmake_minimum_required(VERSION 3.14)
project(ocs2_oc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================
# Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
# ocs2_core's CONFIG_EXTRAS exposes OCS2_CXX_FLAGS / OCS2_CXX_DEFINITIONS used below.

# ============================================================
# Library
# ============================================================
add_library(${PROJECT_NAME}
        src/approximate_model/ChangeOfInputVariables.cpp
        src/approximate_model/LinearQuadraticApproximator.cpp
        src/multiple_shooting/Helpers.cpp
        src/multiple_shooting/Initialization.cpp
        src/multiple_shooting/LagrangianEvaluation.cpp
        src/multiple_shooting/MetricsComputation.cpp
        src/multiple_shooting/PerformanceIndexComputation.cpp
        src/multiple_shooting/ProjectionMultiplierCoefficients.cpp
        src/multiple_shooting/Transcription.cpp
        src/oc_data/LoopshapingPrimalSolution.cpp
        src/oc_data/PerformanceIndex.cpp
        src/oc_data/TimeDiscretization.cpp
        src/oc_problem/OptimalControlProblem.cpp
        src/oc_problem/LoopshapingOptimalControlProblem.cpp
        src/oc_problem/OptimalControlProblemHelperFunction.cpp
        src/oc_problem/OcpSize.cpp
        src/oc_problem/OcpToKkt.cpp
        src/oc_solver/SolverBase.cpp
        src/precondition/Ruzi.cpp
        src/rollout/PerformanceIndicesRollout.cpp
        src/rollout/RolloutBase.cpp
        src/rollout/RootFinder.cpp
        src/rollout/InitializerRollout.cpp
        src/rollout/StateTriggeredRollout.cpp
        src/rollout/TimeTriggeredRollout.cpp
        src/rollout/RolloutSettings.cpp
        src/synchronized_module/ReferenceManager.cpp
        src/synchronized_module/LoopshapingReferenceManager.cpp
        src/synchronized_module/LoopshapingSynchronizedModule.cpp
        src/synchronized_module/SolverObserver.cpp
        src/search_strategy/FilterLinesearch.cpp
        src/trajectory_adjustment/TrajectorySpreading.cpp
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
        ${ocs2_core_TARGETS}
)

# ============================================================
# Install
# ============================================================
install(
        DIRECTORY include/ test/include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

# ============================================================
# Export (deps BEFORE targets so consumers find imported targets first)
# ============================================================
ament_export_dependencies(ocs2_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# ============================================================
# Tests
# ============================================================
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    # All test targets transitively inherit compile flags, include dirs and link
    # libraries (ocs2_core + its public deps) from ${PROJECT_NAME}.

    ament_add_gtest(test_${PROJECT_NAME}_multiple_shooting
            test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
            test/multiple_shooting/testTranscriptionMetrics.cpp
            test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
    )
    target_link_libraries(test_${PROJECT_NAME}_multiple_shooting ${PROJECT_NAME})

    ament_add_gtest(test_${PROJECT_NAME}_data test/oc_data/testTimeDiscretization.cpp)
    target_link_libraries(test_${PROJECT_NAME}_data ${PROJECT_NAME})

    ament_add_gtest(test_${PROJECT_NAME}_rollout
            test/rollout/testTimeTriggeredRollout.cpp
            test/rollout/testStateTriggeredRollout.cpp
    )
    target_link_libraries(test_${PROJECT_NAME}_rollout ${PROJECT_NAME})

    ament_add_gtest(test_change_of_variables test/testChangeOfInputVariables.cpp)
    target_link_libraries(test_change_of_variables ${PROJECT_NAME})

    ament_add_gtest(test_trajectory_spreading test/trajectory_adjustment/TrajectorySpreadingTest.cpp)
    target_link_libraries(test_trajectory_spreading ${PROJECT_NAME})

    ament_add_gtest(test_ocp_to_kkt test/oc_problem/testOcpToKkt.cpp)
    target_link_libraries(test_ocp_to_kkt ${PROJECT_NAME})

    ament_add_gtest(test_precondition test/precondition/testPrecondition.cpp)
    target_link_libraries(test_precondition ${PROJECT_NAME})

endif()

ament_package()
