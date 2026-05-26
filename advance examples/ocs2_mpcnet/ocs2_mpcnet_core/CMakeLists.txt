cmake_minimum_required(VERSION 3.14)
project(ocs2_mpcnet_core)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

find_package(ament_cmake REQUIRED)
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)
find_package(ocs2_python_interface REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(onnxruntime REQUIRED)

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
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_python_interface_TARGETS}
        ${ocs2_ros_interfaces_TARGETS}
        onnxruntime
)

pybind11_add_module(MpcnetPybindings SHARED src/MpcnetPybindings.cpp)
target_link_libraries(MpcnetPybindings PRIVATE ${PROJECT_NAME})

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

install(TARGETS MpcnetPybindings
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_export_dependencies(ocs2_python_interface ocs2_ros_interfaces onnxruntime)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_python_install_package(${PROJECT_NAME})

ament_package()
