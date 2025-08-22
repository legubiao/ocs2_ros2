cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_self_collision)

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

###########
## Build ##
###########

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
        src/PinocchioGeometryInterface.cpp
        src/SelfCollision.cpp
        src/SelfCollisionCppAd.cpp
        src/SelfCollisionConstraint.cpp
        src/SelfCollisionConstraintCppAd.cpp
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
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
