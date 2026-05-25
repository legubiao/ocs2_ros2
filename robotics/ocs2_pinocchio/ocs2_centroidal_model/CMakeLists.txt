cmake_minimum_required(VERSION 3.14)
project(ocs2_centroidal_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
# ocs2_pinocchio_interface exports OCS2_PINOCCHIO_FLAGS via its CONFIG_EXTRAS.

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
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${ocs2_pinocchio_interface_TARGETS}
)

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

ament_export_dependencies(ocs2_robotic_assets ocs2_pinocchio_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test
            test/testAnymalCentroidalModel.cpp
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE test/include)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${ocs2_robotic_assets_TARGETS})
endif()

ament_package()
