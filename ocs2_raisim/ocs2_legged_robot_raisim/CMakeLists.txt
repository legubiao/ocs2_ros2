cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_legged_robot_raisim)




set(dependencies
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_raisim_core
  ocs2_raisim_ros
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_raisim_core REQUIRED)
find_package(ocs2_raisim_ros REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

###########
## Build ##
###########

# Add directories for all targets
include_directories(
  include
)

# main library
add_library(${PROJECT_NAME}
  src/LeggedRobotRaisimConversions.cpp
  src/LeggedRobotRaisimVisualizer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# RaiSim dummy
add_executable(legged_robot_raisim_dummy
  src/LeggedRobotRaisimDummyNode.cpp
)
ament_target_dependencies(legged_robot_raisim_dummy
  ${dependencies}
)
target_link_libraries(legged_robot_raisim_dummy
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_raisim_dummy PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ballbot")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME} legged_robot_raisim_dummy
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

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

install(
  TARGETS 
    legged_robot_raisim_dummy
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

ament_add_gtest(${PROJECT_NAME}_test
  test/testLeggedRobotRaisimConversions.cpp
)
ament_target_dependencies(${PROJECT_NAME}_test
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()