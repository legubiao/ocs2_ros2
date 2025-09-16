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
        );

        /**
         * Destructor
         */
        ~VRMarkerWrapper() = default;

        /**
         * Enable VR control
         */
        void enable();

        /**
         * Disable VR control
         */
        void disable();

        /**
         * Check if VR control is enabled
         * @return true if enabled, false otherwise
         */
        bool isEnabled() const { return enabled_.load(); }

        /**
         * Sync VR position with external position update
         * This method is safe to call even when VR control is disabled
         * @param position New position
         * @param orientation New orientation
         */
        void syncExternalPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    private:
        rclcpp::Node::SharedPtr node_;
        IMarkerControl* markerControl_;
        double linearScale_;
        double angularScale_;
        double updateRate_;
        std::atomic<bool> enabled_;

        /**
         * VR callback function
         * @param msg VR message
         */
        void vrCallback(const char msg);


        /**
         * Update marker position based on VR input
         * @param position New position
         * @param orientation New orientation
         */
        void updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);


        /**
         * Sync current pose with marker position
         * Updates currentPosition_ and currentOrientation_ from marker control
         */
        void syncCurrentPoseWithMarker();


        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr vrSubscriber_;

        // Marker control interface
        IMarkerControl* markerControl_;

        // State management
        std::atomic<bool> enabled_;
        std::mutex stateMutex_;

        // Timing control
        rclcpp::Time lastUpdateTime_; 

        // Current VR position and orientation
        Eigen::Vector3d currentPosition_;
        Eigen::Quaterniond currentOrientation_;

    };

} // namespace ocs2 