cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_thirdparty)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
        DESTINATION "include/${PROJECT_NAME}/../")

ament_export_include_directories(include)

ament_package()
