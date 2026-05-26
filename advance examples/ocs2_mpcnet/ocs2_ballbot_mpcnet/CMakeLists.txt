cmake_minimum_required(VERSION 3.14)
project(ocs2_ballbot_mpcnet)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

find_package(ament_cmake REQUIRED)
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(ocs2_mpcnet_core REQUIRED)
find_package(ocs2_ballbot_ros REQUIRED)

add_library(${PROJECT_NAME}
        src/BallbotMpcnetDefinition.cpp
        src/BallbotMpcnetInterface.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_mpcnet_core_TARGETS}
        ${ocs2_ballbot_ros_TARGETS}
)

pybind11_add_module(BallbotMpcnetPybindings SHARED src/BallbotMpcnetPybindings.cpp)
target_link_libraries(BallbotMpcnetPybindings PRIVATE ${PROJECT_NAME})

add_executable(ballbot_mpcnet_dummy src/BallbotMpcnetDummyNode.cpp)
target_link_libraries(ballbot_mpcnet_dummy ${PROJECT_NAME})

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

install(
        TARGETS
        ballbot_mpcnet_dummy
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS BallbotMpcnetPybindings
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(DIRECTORY launch policy
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ocs2_mpcnet_core ocs2_ballbot_ros)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_python_install_package(${PROJECT_NAME})

ament_package()
