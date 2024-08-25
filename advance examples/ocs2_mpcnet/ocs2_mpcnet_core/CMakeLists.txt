cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_mpcnet_core)

if(NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

set(dependencies
        ocs2_python_interface
        ocs2_ros_interfaces
        onnxruntime
)

find_package(ament_cmake REQUIRED)

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

find_package(ocs2_python_interface REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(onnxruntime REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

# main library
add_library(${PROJECT_NAME}
        src/control/MpcnetBehavioralController.cpp
        src/control/MpcnetOnnxController.cpp
        src/dummy/MpcnetDummyLoopRos.cpp
        src/dummy/MpcnetDummyObserverRos.cpp
        src/rollout/MpcnetDataGeneration.cpp
        src/rollout/MpcnetPolicyEvaluation.cpp
        src/rollout/MpcnetRolloutBase.cpp
        src/rollout/MpcnetRolloutManager.cpp
        src/MpcnetInterfaceBase.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${onnxruntime_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} onnxruntime)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# python bindings
pybind11_add_module(MpcnetPybindings SHARED src/MpcnetPybindings.cpp)
add_dependencies(MpcnetPybindings ${PROJECT_NAME})
target_link_libraries(MpcnetPybindings PRIVATE ${PROJECT_NAME})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_mpcnet_core")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
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

install(TARGETS MpcnetPybindings
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_python_install_package(${PROJECT_NAME})

ament_package()
