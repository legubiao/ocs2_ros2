cmake_minimum_required(VERSION 3.14)
project(ocs2_pinocchio_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_robotic_tools
        pinocchio
        urdfdom
        hpp-fcl
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(urdfdom REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)

# Add pinocchio configurations
include(cmake/pinocchio_config.cmake)

###########
## Build ##
###########

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME} SHARED
        src/PinocchioInterface.cpp
        src/PinocchioInterfaceCppAd.cpp
        src/PinocchioEndEffectorKinematics.cpp
        src/PinocchioEndEffectorKinematicsCppAd.cpp
        src/urdf.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${pinocchio_INCLUDE_DIRS}
        ${hpp-fcl_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

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


############
# Testing ##
############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(testPinocchioInterface
            test/testPinocchioInterface.cpp
            test/testPinocchioEndEffectorKinematics.cpp
    )
    target_link_libraries(testPinocchioInterface ${PROJECT_NAME})
    ament_target_dependencies(testPinocchioInterface ${dependencies})
    target_compile_options(testPinocchioInterface PUBLIC ${OCS2_PINOCCHIO_FLAGS})

endif ()

ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")
