cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_python_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)

find_package(ocs2_ddp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

set(dependencies
        ocs2_ddp
        ocs2_robotic_tools
)


###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/PythonInterface.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

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
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(testDummyPyBindings test/testDummyPyBindings.cpp)
    target_link_libraries(testDummyPyBindings ${PROJECT_NAME})
endif ()
ament_package()