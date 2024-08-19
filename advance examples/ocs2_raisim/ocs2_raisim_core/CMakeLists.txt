cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_raisim_core)

set(CMAKE_PREFIX_PATH "~/raisimLib/raisim/linux" ${CMAKE_PREFIX_PATH})

set(dependencies
        ocs2_oc
        raisim
)

find_package(ament_cmake REQUIRED)

find_package(ocs2_oc REQUIRED)
find_package(raisim REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/RaisimRollout.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${raisim_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


add_executable(${PROJECT_NAME}_lintTarget src/lintTarget.cpp)
ament_target_dependencies(${PROJECT_NAME}_lintTarget ${dependencies})
target_link_libraries(${PROJECT_NAME}_lintTarget
        ${PROJECT_NAME}
        raisim::raisim
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_lintTarget
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
            CF_WERROR
            CT_WERROR
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
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()