cmake_minimum_required(VERSION 3.14)
project(ocs2_sphere_approximation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

# urdfdom 4 vs 5 detection / tinyxml2 are no longer needed: we feed the cached
# raw URDF XML straight into pinocchio::urdf::buildGeom (see
# PinocchioSphereInterface::buildGeomFromPinocchioInterface). This avoids
# urdf::exportURDF, which urdfdom 5.0 (Lyrical+) removed.

add_library(${PROJECT_NAME}
        src/SphereApproximation.cpp
        src/PinocchioSphereInterface.cpp
        src/PinocchioSphereKinematics.cpp
        src/PinocchioSphereKinematicsCppAd.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_pinocchio_interface_TARGETS}
)

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

ament_export_dependencies(ocs2_pinocchio_interface ocs2_robotic_assets)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(PinocchioSphereKinematicsTest test/testPinocchioSphereKinematics.cpp)
    target_link_libraries(PinocchioSphereKinematicsTest ${PROJECT_NAME} ${ocs2_robotic_assets_TARGETS})
endif()

ament_package()
