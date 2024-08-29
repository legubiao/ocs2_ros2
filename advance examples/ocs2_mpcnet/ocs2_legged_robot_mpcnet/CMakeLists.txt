cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_legged_robot_mpcnet)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif ()

set(CMAKE_PREFIX_PATH "~/raisimLib/raisim/linux" ${CMAKE_PREFIX_PATH})

set(dependencies
        ocs2_legged_robot_raisim
        ocs2_mpcnet_core
)

find_package(ament_cmake REQUIRED)
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

find_package(ocs2_mpcnet_core REQUIRED)
find_package(ocs2_legged_robot_raisim REQUIRED)

###########
## Build ##
###########

# main library
add_library(${PROJECT_NAME}
        src/LeggedRobotMpcnetDefinition.cpp
        src/LeggedRobotMpcnetInterface.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# python bindings
pybind11_add_module(LeggedRobotMpcnetPybindings SHARED src/LeggedRobotMpcnetPybindings.cpp)
target_link_libraries(LeggedRobotMpcnetPybindings PRIVATE ${PROJECT_NAME})

# MPC-Net dummy node
add_executable(legged_robot_mpcnet_dummy src/MpcnetDummy.cpp)
target_link_libraries(legged_robot_mpcnet_dummy ${PROJECT_NAME})
target_compile_options(legged_robot_mpcnet_dummy PRIVATE ${OCS2_CXX_FLAGS})

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
        ARCHIVE DESTINATION ib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        legged_robot_mpcnet_dummy
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS LeggedRobotMpcnetPybindings
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(DIRECTORY launch policy
        DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
