cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ocs2)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
# find_package(ocs2_frank_wolfe REQUIRED)
find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


###################################
## ament_cmake specific configuration ##
###################################

set(dependencies
        ocs2_core
        ocs2_ddp
        # ocs2_frank_wolfe
        Boost
)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_lintTarget
        src/lintTarget.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lintTarget
        ${dependencies}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_ocs2")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_lintTarget
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

#############
## Testing ##
#############


ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()