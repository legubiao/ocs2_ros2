cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_msgs)

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MpcState.msg"
        "msg/MpcInput.msg"
        "msg/ModeSchedule.msg"
        "msg/MpcObservation.msg"
        "msg/MpcPerformanceIndices.msg"
        "msg/MpcTargetTrajectories.msg"
        "msg/ControllerData.msg"
        "msg/MpcFlattenedController.msg"
        "msg/LagrangianMetrics.msg"
        "msg/Multiplier.msg"
        "msg/Constraint.msg"
        "srv/Reset.srv"
        DEPENDENCIES std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()