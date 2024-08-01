cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_sqp)

set(dependencies
  ocs2_core
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
  blasfeo_colcon
  hpipm_colcon
  Boost
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(blasfeo_colcon REQUIRED)
find_package(hpipm_colcon REQUIRED)
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
  ${Boost_INCLUDE_DIRS}
)

# Multiple shooting solver library
add_library(${PROJECT_NAME}
  src/SqpLogging.cpp
  src/SqpSettings.cpp
  src/SqpSolver.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies} 
)
target_link_libraries(${PROJECT_NAME}
  hpipm
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
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

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_${PROJECT_NAME}
  test/testCircularKinematics.cpp
  test/testSwitchedProblem.cpp
  test/testUnconstrained.cpp
  test/testValuefunction.cpp
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