cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_raisim_ros)




set(dependencies
  rclcpp
  ocs2_raisim_core
  grid_map_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_raisim_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

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
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/RaisimHeightmapRosConverter.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

#######################
##   CLANG TOOLING   ##
#######################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for project " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
    CT_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
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