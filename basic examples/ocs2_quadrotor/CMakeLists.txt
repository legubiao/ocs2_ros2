cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadrotor)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_ddp
        ocs2_robotic_assets
        ocs2_python_interface
)

find_package(ament_cmake REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_python_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Quadrotor interface library
add_library(${PROJECT_NAME}
        src/QuadrotorInterface.cpp
        src/QuadrotorSystemDynamics.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# python bindings
pybind11_add_module(QuadrotorPyBindings src/pyBindModule.cpp)
target_link_libraries(QuadrotorPyBindings PRIVATE
        ${PROJECT_NAME}
        dl
)
set_target_properties(QuadrotorPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin"
)

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
install(TARGETS QuadrotorPyBindings
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)
    ament_add_gtest(${PROJECT_NAME}_PyBindingsTest test/PyBindingsTest.cpp)
    target_include_directories(${PROJECT_NAME}_PyBindingsTest PRIVATE ${PROJECT_BINARY_DIR}/include)
    ament_target_dependencies(${PROJECT_NAME}_PyBindingsTest ${dependencies})
    target_link_libraries(${PROJECT_NAME}_PyBindingsTest ${PROJECT_NAME})

endif ()

ament_package()