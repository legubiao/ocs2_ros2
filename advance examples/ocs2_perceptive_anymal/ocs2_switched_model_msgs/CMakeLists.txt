cmake_minimum_required(VERSION 3.14)
project(ocs2_switched_model_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Gait.msg"
        "msg/GaitSequence.msg"
        "msg/ScheduledGaitSequence.msg"
        "srv/TrajectoryRequest.srv"
        DEPENDENCIES std_msgs ocs2_msgs
)

ament_package()
