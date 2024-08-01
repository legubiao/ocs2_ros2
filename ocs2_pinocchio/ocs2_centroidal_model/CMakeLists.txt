cmake_minimum_required(VERSION 3.14)
project(ocs2_centroidal_model)


set(dependencies
        ocs2_core
        ocs2_robotic_tools
        ocs2_robotic_assets
        ocs2_pinocchio_interface
        hpp-fcl
        pinocchio
        urdf
        Boost
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(pinocchio REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log_setup
        log
)

###########
## Build ##
###########

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
        include
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/PinocchioCentroidalDynamics.cpp
        src/PinocchioCentroidalDynamicsAD.cpp
        src/CentroidalModelRbdConversions.cpp
        src/CentroidalModelPinocchioMapping.cpp
        src/CentroidalModelInfo.cpp
        src/FactoryFunctions.cpp
        src/ModelHelperFunctions.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

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

ament_add_gtest(${PROJECT_NAME}_test
        # test/testAccessMethods.cpp
        test/testAnymalCentroidalModel.cpp
)
ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
target_include_directories(${PROJECT_NAME}_test PRIVATE
        test/include
)
target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")
