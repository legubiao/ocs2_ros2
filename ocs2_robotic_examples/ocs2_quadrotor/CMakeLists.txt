cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadrotor)


set(dependencies
        ocs2_mpc
        ocs2_ddp
        ocs2_robotic_tools
        ocs2_robotic_assets
        ocs2_python_interface
        Boost
)
find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_python_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Add directories for all targets
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# Quadrotor interface library
add_library(${PROJECT_NAME}
        src/QuadrotorInterface.cpp
        src/QuadrotorSystemDynamics.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# python bindings
pybind11_add_module(QuadrotorPyBindings
        src/pyBindModule.cpp
)
target_link_libraries(QuadrotorPyBindings PRIVATE
        ${PROJECT_NAME}
        dl
)
set_target_properties(QuadrotorPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin"
)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_quadrotor")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
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
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS QuadrotorPyBindings
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

ament_add_gtest(${PROJECT_NAME}_PyBindingsTest
        test/PyBindingsTest.cpp
)
target_include_directories(${PROJECT_NAME}_PyBindingsTest
        PRIVATE ${PROJECT_BINARY_DIR}/include
)
ament_target_dependencies(${PROJECT_NAME}_PyBindingsTest
        ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_PyBindingsTest
        ${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()