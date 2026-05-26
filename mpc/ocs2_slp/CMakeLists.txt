cmake_minimum_required(VERSION 3.14)
project(ocs2_slp)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)

add_library(${PROJECT_NAME}
        src/pipg/PipgSettings.cpp
        src/pipg/PipgSolver.cpp
        src/pipg/SingleThreadPipg.cpp
        src/Helpers.cpp
        src/SlpSettings.cpp
        src/SlpSolver.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
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

ament_export_dependencies(ocs2_mpc ocs2_qp_solver)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME}
            test/testHelpers.cpp
            test/testPipgSolver.cpp
            test/testSlpSolver.cpp
    )
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_package()
