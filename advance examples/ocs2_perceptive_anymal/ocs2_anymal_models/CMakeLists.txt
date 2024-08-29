cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_anymal_models)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)

find_package(ocs2_switched_model_interface REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

set(dependencies
        ocs2_switched_model_interface
        ocs2_pinocchio_interface
)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

add_library(${PROJECT_NAME}
        src/AnymalModels.cpp
        src/FrameDeclaration.cpp
        src/QuadrupedPinocchioMapping.cpp
        src/QuadrupedInverseKinematics.cpp
        src/QuadrupedKinematics.cpp
        src/QuadrupedCom.cpp
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

install(
        DIRECTORY config launch urdf
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)

    find_package(ament_cmake_gtest)

    # Build unit tests
    ament_add_gtest(${PROJECT_NAME}_switched_model_test test/TestDynamicsHelpers.cpp)
    ament_target_dependencies(${PROJECT_NAME}_switched_model_test ${dependencies})
    target_link_libraries(${PROJECT_NAME}_switched_model_test ${PROJECT_NAME})

    ament_add_gtest(TestQuadrupedPinocchio
            test/camel/AnymalCamelCom.cpp
            test/camel/AnymalCamelKinematics.cpp
            test/TestFrameDeclaration.cpp
            test/TestInverseKinematics.cpp
            test/TestQuadrupedPinocchioCom.cpp
            test/TestQuadrupedPinocchioKinematics.cpp
    )
    ament_target_dependencies(TestQuadrupedPinocchio ${dependencies})
    target_link_libraries(TestQuadrupedPinocchio ${PROJECT_NAME})

endif ()
ament_package()