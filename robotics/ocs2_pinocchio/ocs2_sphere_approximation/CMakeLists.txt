cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_sphere_approximation)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_pinocchio_interface
)

# Include the FindPkgConfig module
include(FindPkgConfig)

# Use pkg_check_modules to find urdfdom and get its version
pkg_check_modules(URDFDOM urdfdom)

if (URDFDOM_VERSION VERSION_GREATER_EQUAL 4.0.0)
    add_definitions(-DURDFDOM_VERSION_GT_4)
    find_package(tinyxml2 REQUIRED)
    list(APPEND dependencies tinyxml2)
endif ()

find_package(ament_cmake REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

###########
## Build ##
###########

# ocs2 sphere approximation library
add_library(${PROJECT_NAME}
        src/SphereApproximation.cpp
        src/PinocchioSphereInterface.cpp
        src/PinocchioSphereKinematics.cpp
        src/PinocchioSphereKinematicsCppAd.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
if (URDFDOM_VERSION VERSION_GREATER_EQUAL 4.0.0)
    target_link_libraries(${PROJECT_NAME} tinyxml2)
endif ()
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

############
# Testing ##
############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(PinocchioSphereKinematicsTest test/testPinocchioSphereKinematics.cpp)
    ament_target_dependencies(PinocchioSphereKinematicsTest ${dependencies} ocs2_robotic_assets)
    target_link_libraries(PinocchioSphereKinematicsTest ${PROJECT_NAME} ${COAL_LIBRARIES})
    target_compile_options(PinocchioSphereKinematicsTest PUBLIC ${FLAGS})

endif ()

ament_package()
