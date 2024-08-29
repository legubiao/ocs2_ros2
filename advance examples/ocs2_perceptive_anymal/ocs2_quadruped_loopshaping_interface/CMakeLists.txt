cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadruped_loopshaping_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## ament_cmake Dependencies
set(dependencies
        ocs2_quadruped_interface
)

find_package(ament_cmake REQUIRED)

find_package(ocs2_quadruped_interface REQUIRED)


###########
## Build ##
###########
# Declare a C++ library
add_library(${PROJECT_NAME}
        src/QuadrupedLoopshapingDummyNode.cpp
        src/QuadrupedLoopshapingInterface.cpp
        src/QuadrupedLoopshapingMpc.cpp
        src/QuadrupedLoopshapingMpcNode.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
