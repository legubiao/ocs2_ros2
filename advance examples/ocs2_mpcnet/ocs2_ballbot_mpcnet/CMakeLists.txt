cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_ballbot_mpcnet)


set(dependencies
        ocs2_ballbot_ros
        ocs2_mpcnet_core
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_mpcnet_core REQUIRED)
find_package(ocs2_ballbot_ros REQUIRED)

find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

# main library
add_library(${PROJECT_NAME}
        src/BallbotMpcnetDefinition.cpp
        src/BallbotMpcnetInterface.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# python bindings
pybind11_add_module(BallbotMpcnetPybindings SHARED src/BallbotMpcnetPybindings.cpp)
target_link_libraries(BallbotMpcnetPybindings PRIVATE ${PROJECT_NAME})

# MPC-Net dummy node
add_executable(ballbot_mpcnet_dummy src/BallbotMpcnetDummyNode.cpp)
ament_target_dependencies(ballbot_mpcnet_dummy ${dependencies})
target_link_libraries(ballbot_mpcnet_dummy ${PROJECT_NAME})
target_compile_options(ballbot_mpcnet_dummy PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_ballbot_mpcnet")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} ballbot_mpcnet_dummy
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
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
        ARCHIVE DESTINATION ib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        TARGETS BallbotMpcnetPybindings
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        ballbot_mpcnet_dummy
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch policy
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
