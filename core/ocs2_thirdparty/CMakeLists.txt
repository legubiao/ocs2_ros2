cmake_minimum_required(VERSION 3.16)
project(ocs2_thirdparty)

find_package(ament_cmake REQUIRED)

# Header-only INTERFACE library bundling the third-party headers shipped under include/.
# Exported as ocs2_thirdparty::ocs2_thirdparty so downstream packages can use either
# target_link_libraries(<tgt> PUBLIC ocs2_thirdparty::ocs2_thirdparty) or rely on
# ${ocs2_thirdparty_TARGETS} populated by find_package(ocs2_thirdparty).
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

install(
        DIRECTORY include/
        DESTINATION include/
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
