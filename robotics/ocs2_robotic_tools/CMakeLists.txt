cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_robotic_tools)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_oc REQUIRED)

set(dependencies
        ocs2_oc
)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/common/RotationTransforms.cpp
        src/common/LoopshapingRobotInterface.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(rotation_transform_tests
            test/common/TestRotationTransforms.cpp
            test/common/TestRotationDerivativesTransforms.cpp
    )
    target_link_libraries(rotation_transform_tests ${PROJECT_NAME})
    ament_target_dependencies(rotation_transform_tests ${dependencies})
    target_compile_options(rotation_transform_tests PRIVATE ${OCS2_CXX_FLAGS})
endif ()
ament_package()