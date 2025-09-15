#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include <map>
#include <string>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {


    class VRMarkerWrapper {
    public:
        /**
         * Constructor
         * @param node ROS node handle
         * @param markerControl Pointer to marker control interface
         * @param linearScale Scale factor for linear movement
         * @param angularScale Scale factor for angular movement
         * @param updateRate Update rate for joystick processing (Hz)
         */        
        VRMarkerWrapper(
            rclcpp::Node::SharedPtr node,
            IMarkerControl* markerControl,
            const double linearScale = 0.1,
            const double angularScale = 0.1,
            const double updateRate = 30.0,
        )
            

}