cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project (ocs2_frank_wolfe)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

find_package(ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(GLPK MODULE REQUIRED)

###########
## Build ##
###########

## Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS} 
)

set(dependencies
    ocs2_core
    GLPK
)

add_library(${PROJECT_NAME}
  src/FrankWolfeDescentDirection.cpp
  src/GradientDescent.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(quadratic_test
  test/QuadraticTest.cpp
)
ament_target_dependencies(quadratic_test ${dependencies})
target_link_libraries(quadratic_test
  ${PROJECT_NAME}
  GLPK::GLPK
)

ament_add_gtest(matyas_test
  test/MatyasTest.cpp
)
ament_target_dependencies(matyas_test ${dependencies})
target_link_libraries(matyas_test
  ${PROJECT_NAME}
  GLPK::GLPK
)

ament_add_gtest(glpk_test
  test/testGLPK.cpp
)
ament_target_dependencies(glpk_test ${dependencies})
target_link_libraries(glpk_test
  ${PROJECT_NAME}
  GLPK::GLPK
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
