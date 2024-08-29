cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ipm)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_mpc
        blasfeo_colcon
        hpipm_colcon
)

find_package(ament_cmake REQUIRED)

find_package(ocs2_mpc REQUIRED)
find_package(blasfeo_colcon REQUIRED)
find_package(hpipm_colcon REQUIRED)

###########
## Build ##
###########
# Multiple shooting solver library
add_library(${PROJECT_NAME}
        src/IpmHelpers.cpp
        src/IpmInitialization.cpp
        src/IpmPerformanceIndexComputation.cpp
        src/IpmSettings.cpp
        src/IpmSolver.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} hpipm)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# #########################
# ###   CLANG TOOLING   ###
# #########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include test
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

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

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME}
            test/Exp0Test.cpp
            test/Exp1Test.cpp
            test/testCircularKinematics.cpp
            test/testSwitchedProblem.cpp
            test/testUnconstrained.cpp
            test/testValuefunction.cpp
    )
    ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

endif ()

ament_package()