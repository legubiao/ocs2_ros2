cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_anymal_mpc)

find_package(ament_cmake REQUIRED)

find_package(grid_map_sdf REQUIRED)
find_package(ocs2_anymal_models REQUIRED)
find_package(ocs2_quadruped_interface REQUIRED)

set(dependencies
        grid_map_sdf
        ocs2_anymal_models
        ocs2_quadruped_interface
)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/AnymalInterface.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


add_executable(${PROJECT_NAME}_mpc_node src/AnymalMpcNode.cpp)
ament_target_dependencies(${PROJECT_NAME}_mpc_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_mpc_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_dummy_mrt_node src/AnymalDummyMRT.cpp)
ament_target_dependencies(${PROJECT_NAME}_dummy_mrt_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node ${PROJECT_NAME})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_mpc_node ${PROJECT_NAME}_dummy_mrt_node
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
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
)
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

install(
        TARGETS
        ${PROJECT_NAME}_mpc_node
        ${PROJECT_NAME}_dummy_mrt_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest)

    # Build unit tests
    ament_add_gtest(${PROJECT_NAME}_test test/testProblemFormulation.cpp)
    ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

endif ()

ament_package()