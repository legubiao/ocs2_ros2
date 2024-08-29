cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_qp_solver)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_oc REQUIRED)

set(dependencies
        ocs2_oc
)

add_library(${PROJECT_NAME}
        src/Ocs2QpSolver.cpp
        src/QpDiscreteTranscription.cpp
        src/QpSolver.cpp
        src/QpTrajectories.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
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
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
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

    ament_add_gtest(test_${PROJECT_NAME}
            test/testDiscreteTranscription.cpp
            test/testQpSolver.cpp
            test/testOcs2QpSolver.cpp
    )
    ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

endif ()
ament_package()
