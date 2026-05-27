cmake_minimum_required(VERSION 3.14)
project(ocs2_pinocchio_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(urdfdom REQUIRED)
# pinocchio brings in its collision backend (hpp-fcl on Jazzy, coal on Lyrical+)
# transitively via INTERFACE_LINK_LIBRARIES and find_dependency, so OCS2 does
# not need to find/link coal or hpp-fcl explicitly. See collision_compat.h for
# the C++-side namespace alias.
find_package(pinocchio REQUIRED)

# pinocchio compile flags (-DPINOCCHIO_* defines + -Wno-* suppressions).
include(cmake/pinocchio_config.cmake)

# Strip octomap from the collision backend's INTERFACE_LINK_LIBRARIES to avoid
# the multi-octomap ABI conflict (see cmake/octomap_cleanup.cmake for the full
# rationale). MUST be included AFTER find_package(pinocchio) so the imported
# targets already exist.
include(cmake/octomap_cleanup.cmake)

add_library(${PROJECT_NAME} SHARED
        src/PinocchioInterface.cpp
        src/PinocchioInterfaceCppAd.cpp
        src/PinocchioEndEffectorKinematics.cpp
        src/PinocchioEndEffectorKinematicsCppAd.cpp
        src/urdf.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
# pinocchio::pinocchio publishes BOOST_MPL_LIMIT_VECTOR_SIZE=30 in its
# INTERFACE_COMPILE_DEFINITIONS, which conflicts with GCC's preprocessed Boost.MPL
# (hardcoded to 20 template args). The conflict breaks any consumer that also
# includes boost/multi_index or boost/property_tree (e.g. ocs2_legged_robot).
# Disabling MPL's preprocessed headers forces use of the real templates that
# honour the increased limits, restoring co-compatibility.
target_compile_definitions(${PROJECT_NAME} PUBLIC
        ${OCS2_CXX_DEFINITIONS}
        BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_robotic_tools_TARGETS}
        ${urdfdom_TARGETS}
        pinocchio::pinocchio
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

ament_export_dependencies(ocs2_robotic_tools urdfdom pinocchio)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(testPinocchioInterface
            test/testPinocchioInterface.cpp
            test/testPinocchioEndEffectorKinematics.cpp
    )
    target_link_libraries(testPinocchioInterface ${PROJECT_NAME})
endif()

ament_package(
        CONFIG_EXTRAS "cmake/pinocchio_config.cmake"
        CONFIG_EXTRAS_POST "cmake/octomap_cleanup.cmake"
)
