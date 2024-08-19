cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_raisim_ros)

set(CMAKE_PREFIX_PATH "~/raisimLib/raisim/linux" ${CMAKE_PREFIX_PATH})

set(dependencies
        rclcpp
        ocs2_raisim_core
        grid_map_msgs
)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ocs2_raisim_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/RaisimHeightmapRosConverter.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#######################
##   CLANG TOOLING   ##
#######################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for project " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
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


#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

ament_add_gtest(test_${PROJECT_NAME}
        test/testRosConversion.cpp
)
ament_target_dependencies(test_${PROJECT_NAME}
        ${dependencies}
)
target_link_libraries(test_${PROJECT_NAME}
        ${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()