cmake_minimum_required(VERSION 3.14)
project(ocs2_ballbot)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

find_package(ament_cmake REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_slp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_python_interface REQUIRED)

configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

add_library(${PROJECT_NAME}
        src/BallbotInterface.cpp
        src/dynamics/BallbotSystemDynamics.cpp
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
        ${ocs2_slp_TARGETS}
        ${ocs2_sqp_TARGETS}
        ${ocs2_robotic_tools_TARGETS}
        ${ocs2_python_interface_TARGETS}
)

pybind11_add_module(BallbotPyBindings SHARED src/pyBindModule.cpp)
target_link_libraries(BallbotPyBindings PRIVATE ${PROJECT_NAME})

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

install(TARGETS BallbotPyBindings
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ocs2_ddp ocs2_slp ocs2_sqp ocs2_robotic_tools ocs2_python_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)

    ament_add_gtest(test_BallbotPyBindings test/testBallbotPyBindings.cpp)
    target_link_libraries(test_BallbotPyBindings ${PROJECT_NAME})
endif()

ament_package()
