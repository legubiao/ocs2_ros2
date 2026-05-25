cmake_minimum_required(VERSION 3.14)
project(ocs2_quadrotor)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_python_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

add_library(${PROJECT_NAME}
        src/QuadrotorInterface.cpp
        src/QuadrotorSystemDynamics.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_ddp_TARGETS}
        ${ocs2_python_interface_TARGETS}
        ${ocs2_robotic_assets_TARGETS}
)

pybind11_add_module(QuadrotorPyBindings src/pyBindModule.cpp)
target_link_libraries(QuadrotorPyBindings PRIVATE
        ${PROJECT_NAME}
        dl
)
set_target_properties(QuadrotorPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin"
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        FILES "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h"
        DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
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

ament_export_dependencies(ocs2_ddp ocs2_python_interface ocs2_robotic_assets)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(${PROJECT_NAME}_PyBindingsTest test/PyBindingsTest.cpp)
    target_link_libraries(${PROJECT_NAME}_PyBindingsTest ${PROJECT_NAME})
endif()

ament_package()
