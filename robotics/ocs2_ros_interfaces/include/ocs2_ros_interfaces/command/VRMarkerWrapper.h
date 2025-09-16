#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include <map>
#include <string>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    const std::string XR_NODE_NAME = "xr_target_node";

    class VRMarkerWrapper {
    public:
        /**
         * Constructor
         * @param node ROS node handle
         * @param markerControl Pointer to marker control interface
         * @param updateRate Update rate for joystick processing (Hz)
         */        
        VRMarkerWrapper(
            rclcpp::Node::SharedPtr node,
            IMarkerControl* markerControl,
            const double updateRate = 30.0,
        );

        /**
         * Destructor
         */
        ~VRMarkerWrapper() = default;

        bool check_node_exists(const std::shared_ptr<rclcpp::Node>& node, const std::string& target_node_name);

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
        // void syncExternalPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);


        // Eigen::Matrix4d VRMarkerWrapper::getLeftPose() const;
        // Eigen::Matrix4d VRMarkerWrapper::getRightPose() const;

    private:
        rclcpp::Node::SharedPtr node_;
        IMarkerControl* markerControl_;
        double updateRate_;
        std::atomic<bool> enabled_;

        /**
         * VR callback function
         * @param msg VR message
         */
        void vrLeftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void vrRightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


        /**
         * Update marker position based on VR input
         * @param position New position
         * @param orientation New orientation
         * @param targetArm Target arm for dual arm mode
         */
        void updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const ArmType targetArm);


        /**
         * Sync current pose with marker position
         * Updates currentPosition_ and currentOrientation_ from marker control
         */
        // void syncCurrentPoseWithMarker();

        /**
         * Convert PoseStamped message to Eigen::Matrix4d
         * @param msg PoseStamped message
         * @return 4x4 transformation matrix
         */
        Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void matrixToPosOri(const Eigen::Matrix4d& matrix, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subLeft_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subRight_;

        // VR ee pose matrix parameters
        Eigen::Matrix4d leftEEPose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d rightEEPose_ = Eigen::Matrix4d::Identity();


        // VR position and orientation parameters
        Eigen::Vector3d leftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond leftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d rightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rightOrientation_ = Eigen::Quaterniond::Identity();


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