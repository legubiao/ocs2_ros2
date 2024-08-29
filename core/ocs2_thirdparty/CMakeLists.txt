cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_thirdparty)

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

# Install headers
install(
        DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)

ament_package()
