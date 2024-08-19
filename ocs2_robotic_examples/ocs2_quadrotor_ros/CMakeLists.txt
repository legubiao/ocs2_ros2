cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadrotor_ros)


set(dependencies
        rclcpp
        tf2_ros
        ocs2_ros_interfaces
        ocs2_quadrotor
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_quadrotor REQUIRED)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
        src/QuadrotorDummyVisualization.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# MPC node
add_executable(quadrotor_mpc src/QuadrotorMpcNode.cpp)
target_link_libraries(quadrotor_mpc ${PROJECT_NAME})
ament_target_dependencies(quadrotor_mpc ${dependencies})

# Dummy node
add_executable(quadrotor_dummy_test
        src/DummyQuadrotorNode.cpp
        src/QuadrotorDummyVisualization.cpp
)
target_link_libraries(quadrotor_dummy_test ${PROJECT_NAME})
ament_target_dependencies(quadrotor_dummy_test ${dependencies})

# Target node
add_executable(quadrotor_target src/QuadrotorTargetPoseCommand.cpp)
target_link_libraries(quadrotor_target ${PROJECT_NAME})
ament_target_dependencies(quadrotor_target ${dependencies})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_quadrotor")
    add_clang_tooling(
            TARGETS
            quadrotor_mpc
            quadrotor_dummy_test
            quadrotor_target
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
        TARGETS
        quadrotor_mpc
        quadrotor_dummy_test
        quadrotor_target
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(
        DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()