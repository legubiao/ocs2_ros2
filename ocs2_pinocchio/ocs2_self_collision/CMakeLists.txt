cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_self_collision)


set(dependencies
        ocs2_core
        ocs2_robotic_tools
        ocs2_pinocchio_interface
        hpp-fcl
        pinocchio
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
# requires liboctomap-dev and libassimp-dev
find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${hpp-fcl_INCLUDE_DIRS}
)

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
        src/PinocchioGeometryInterface.cpp
        src/SelfCollision.cpp
        src/SelfCollisionCppAd.cpp
        src/SelfCollisionConstraint.cpp
        src/SelfCollisionConstraintCppAd.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
