cmake_minimum_required(VERSION 3.14)
project(ocs2_quadruped_loopshaping_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_quadruped_interface REQUIRED)

add_library(${PROJECT_NAME}
        src/QuadrupedLoopshapingDummyNode.cpp
        src/QuadrupedLoopshapingInterface.cpp
        src/QuadrupedLoopshapingMpc.cpp
        src/QuadrupedLoopshapingMpcNode.cpp
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
        ${ocs2_quadruped_interface_TARGETS}
)

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

ament_export_dependencies(ocs2_quadruped_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
