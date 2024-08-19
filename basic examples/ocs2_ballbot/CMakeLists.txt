cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ballbot)


set(dependencies
        ocs2_ddp
        ocs2_slp
        ocs2_sqp
        ocs2_robotic_tools
        ocs2_python_interface
)

find_package(ament_cmake REQUIRED)

find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_slp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_python_interface REQUIRED)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Ballbot interface library
add_library(${PROJECT_NAME}
        src/BallbotInterface.cpp
        src/dynamics/BallbotSystemDynamics.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# python bindings
pybind11_add_module(BallbotPyBindings SHARED src/pyBindModule.cpp)
target_link_libraries(BallbotPyBindings PUBLIC ${PROJECT_NAME})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_ballbot")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
            CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
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
        ARCHIVE DESTINATION ib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS BallbotPyBindings
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
    find_package(ament_cmake_gtest)

    ament_add_gtest(test_BallbotPyBindings test/testBallbotPyBindings.cpp)
    target_include_directories(test_BallbotPyBindings PRIVATE ${PROJECT_BINARY_DIR}/include)
    ament_target_dependencies(test_BallbotPyBindings ${dependencies})
    target_link_libraries(test_BallbotPyBindings ${PROJECT_NAME})

endif ()

ament_package()
