cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_sphere_approximation)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_pinocchio_interface
        ocs2_robotic_assets
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

###########
## Build ##
###########

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -DPINOCCHIO_WITH_HPP_FCL
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

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

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
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
    ament_target_dependencies(PinocchioSphereKinematicsTest ${dependencies})
    target_link_libraries(PinocchioSphereKinematicsTest ${PROJECT_NAME})
    target_compile_options(PinocchioSphereKinematicsTest PUBLIC ${FLAGS})
endif ()

ament_package()
