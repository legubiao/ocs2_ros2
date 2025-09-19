#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include <map>
#include <string>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    const std::string XR_NODE_NAME = "/xr_target_node";
    
    // Thresholds for pose change detection
    const double POSITION_THRESHOLD = 0.01;  // 1mm threshold for position changes
    const double ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)

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
            const double updateRate = 500.0);

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

    private:
        double updateRate_;

        /**
         * VR callback function
         * @param msg VR message
         */
        void vrLeftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void vrRightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * Right thumbstick callback function
         * @param msg Boolean message indicating thumbstick press
         */
        void rightThumbstickCallback(const std_msgs::msg::Bool::SharedPtr msg);

        /**
         * Robot current pose callback functions
         * @param msg Robot current pose message
         */
        void robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


        /**
         * Update marker position based on VR input
         * @param position New position
         * @param orientation New orientation
         * @param targetArm Target arm for dual arm mode
         */
        void updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const IMarkerControl::ArmType targetArm);



        /**
         * Convert PoseStamped message to Eigen::Matrix4d
         * @param msg PoseStamped message
         * @return 4x4 transformation matrix
         */
        Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void matrixToPosOri(const Eigen::Matrix4d& matrix, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

        /**
         * Check if pose has changed significantly
         * @param currentPos Current position
         * @param currentOri Current orientation
         * @param prevPos Previous position
         * @param prevOri Previous orientation
         * @return true if pose has changed significantly
         */
        bool hasPoseChanged(const Eigen::Vector3d& currentPos, const Eigen::Quaterniond& currentOri,
                           const Eigen::Vector3d& prevPos, const Eigen::Quaterniond& prevOri);

        /**
         * Calculate pose difference and apply to robot base pose
         * @param vrCurrentPos Current VR position
         * @param vrCurrentOri Current VR orientation
         * @param vrBasePos VR base position
         * @param vrBaseOri VR base orientation
         * @param robotBasePos Robot base position
         * @param robotBaseOri Robot base orientation
         * @param resultPos Output calculated position
         * @param resultOri Output calculated orientation
         */
        void calculatePoseFromDifference(const Eigen::Vector3d& vrCurrentPos, const Eigen::Quaterniond& vrCurrentOri,
                                       const Eigen::Vector3d& vrBasePos, const Eigen::Quaterniond& vrBaseOri,
                                       const Eigen::Vector3d& robotBasePos, const Eigen::Quaterniond& robotBaseOri,
                                       Eigen::Vector3d& resultPos, Eigen::Quaterniond& resultOri);

        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subLeft_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subRight_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subRightThumbstick_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subRobotLeftPose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subRobotRightPose_;

        // VR ee pose matrix parameters
        Eigen::Matrix4d leftEEPose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d rightEEPose_ = Eigen::Matrix4d::Identity();


        // VR position and orientation parameters
        Eigen::Vector3d leftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond leftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d rightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rightOrientation_ = Eigen::Quaterniond::Identity();

        // Previous pose for change detection (update mode)
        Eigen::Vector3d prevCalculatedLeftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prevCalculatedLeftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prevCalculatedRightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prevCalculatedRightOrientation_ = Eigen::Quaterniond::Identity();

        // Previous VR pose for change detection (storage mode)
        Eigen::Vector3d prevVRLeftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prevVRLeftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prevVRRightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prevVRRightOrientation_ = Eigen::Quaterniond::Identity();

        // State management
        std::atomic<bool> isUpdateMode_;  // true = update mode, false = storage mode
        std::atomic<bool> lastThumbstickState_;

        // VR base poses (stored when thumbstick is pressed)
        Eigen::Vector3d vrBaseLeftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vrBaseLeftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d vrBaseRightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vrBaseRightOrientation_ = Eigen::Quaterniond::Identity();

        // Robot base poses (stored when thumbstick is pressed)
        Eigen::Vector3d robotBaseLeftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robotBaseLeftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robotBaseRightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robotBaseRightOrientation_ = Eigen::Quaterniond::Identity();

        // Current robot poses
        Eigen::Vector3d robotCurrentLeftPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robotCurrentLeftOrientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robotCurrentRightPosition_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robotCurrentRightOrientation_ = Eigen::Quaterniond::Identity();


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