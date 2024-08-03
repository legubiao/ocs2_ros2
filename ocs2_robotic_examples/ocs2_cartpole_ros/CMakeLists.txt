cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_cartpole_ros)


set(dependencies
        rclcpp
        ocs2_core
        ocs2_ddp
        ocs2_mpc
        ocs2_ros_interfaces
        ocs2_robotic_tools
        ocs2_cartpole
        ocs2_robotic_assets
        Boost
        ament_index_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_cartpole REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/CartpoleMpcNode.cpp
        src/DummyCartpoleNode.cpp
        src/CartpoleDummyVisualization.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(cartpole_mpc src/CartpoleMpcNode.cpp)
ament_target_dependencies(cartpole_mpc ${dependencies})
target_link_libraries(cartpole_mpc ${PROJECT_NAME})

# Dummy node
add_executable(cartpole_dummy_test
        src/DummyCartpoleNode.cpp
        src/CartpoleDummyVisualization.cpp
)
ament_target_dependencies(cartpole_dummy_test ${dependencies})
target_link_libraries(cartpole_dummy_test ${PROJECT_NAME})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Running clang tooling.")
    add_clang_tooling(
            TARGETS
            cartpole_mpc
            cartpole_dummy_test
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
        TARGETS
        cartpole_mpc
        cartpole_dummy_test
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
