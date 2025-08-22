cmake_minimum_required(VERSION 3.14)
project(ocs2_centroidal_model)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_robotic_assets
        ocs2_pinocchio_interface
)

find_package(ament_cmake REQUIRED)

find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
        src/PinocchioCentroidalDynamics.cpp
        src/PinocchioCentroidalDynamicsAD.cpp
        src/CentroidalModelRbdConversions.cpp
        src/CentroidalModelPinocchioMapping.cpp
        src/CentroidalModelInfo.cpp
        src/FactoryFunctions.cpp
        src/ModelHelperFunctions.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

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
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test
            # test/testAccessMethods.cpp
            test/testAnymalCentroidalModel.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
    target_include_directories(${PROJECT_NAME}_test PRIVATE test/include)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})

endif ()

ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")
