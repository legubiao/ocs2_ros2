#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    /**
     * Marker auto position wrapper that automatically updates marker positions based on end effector poses.
     * This class provides automatic marker position synchronization with robot end effector positions.
     * Supports both single arm and dual arm modes.
     */
    class MarkerAutoPositionWrapper {
    public:
        /**
         * Auto position update modes
         */
        enum class UpdateMode {
            DISABLED,           // Disable auto updates
            INITIALIZATION,     // Update only during initialization
            CONTINUOUS          // Continuous updates (with cooldown, paused on MPC observation)
        };

        /**
         * Constructor
         * @param node ROS node
         * @param topicPrefix Topic prefix
         * @param markerControl Marker control interface
         * @param updateMode Update mode
         * @param cooldownDuration Cooldown duration (seconds)
         * @param maxUpdateFrequency Maximum update frequency (Hz)
         * @param dualArmMode Whether to enable dual arm mode
         */
        MarkerAutoPositionWrapper(
            rclcpp::Node::SharedPtr node,
            std::string  topicPrefix,
            IMarkerControl* markerControl,
            UpdateMode updateMode = UpdateMode::INITIALIZATION,
            bool dualArmMode = false,
            double cooldownDuration = 3.0,
            double maxUpdateFrequency = 1.0);

        /**
         * Destructor
         */
        ~MarkerAutoPositionWrapper() = default;

    private:
        /**
         * Left arm end effector pose callback (dual arm mode)
         * @param msg Pose message
         */
        void leftEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        /**
         * Right arm end effector pose callback (dual arm mode)
         * @param msg Pose message
         */
        void rightEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        /**
         * MPC observation callback
         * @param msg MPC observation message
         */
        void observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg);

        /**
         * Cooldown check callback
         */
        void checkCooldownCallback();

        /**
         * Update left arm marker position
         * @param msg Left arm pose message
         */
        void updateLeftArmMarkerPosition(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        /**
         * Update right arm marker position
         * @param msg Right arm pose message
         */
        void updateRightArmMarkerPosition(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        /**
         * Common pose callback logic
         * @param msg Pose message
         * @param isLeftArm Whether this is for left arm
         * @return true if should update
         */
        bool processPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg, bool isLeftArm);

        /**
         * Common marker update logic
         * @param msg Pose message
         * @param armType Arm type (LEFT or RIGHT)
         * @param markerName Marker name for display
         */
        void updateMarkerPosition(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg, 
                                 IMarkerControl::ArmType armType, 
                                 const std::string& markerName);

        /**
         * Handle initialization logic
         * @param isLeftArm Whether this is for left arm
         */
        void handleInitialization(bool isLeftArm);

        /**
         * Handle INITIALIZATION mode logic
         */
        void handleInitializationMode();

        /**
         * Check if position should be updated
         * @return true if should update, false otherwise
         */
        bool shouldUpdatePosition() const;

        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leftEndEffectorPoseSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rightEndEffectorPoseSubscriber_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
        rclcpp::TimerBase::SharedPtr cooldownCheckTimer_;

        // Marker control interface
        IMarkerControl* markerControl_;

        // Configuration parameters
        std::string topicPrefix_;
        UpdateMode updateMode_;
        double cooldownDuration_;
        double maxUpdateFrequency_;
        double minUpdateInterval_;  // Minimum update interval (seconds)
        bool dualArmMode_;

        bool initialized_;
        bool updateEnabled_;
        rclcpp::Time lastMpcObservationTime_;
        rclcpp::Time lastUpdateTime_;  // Last update time

        // Dual arm mode state
        bool leftArmPoseReceived_;
        bool rightArmPoseReceived_;

        // Thread safety
        mutable std::mutex stateMutex_;
    };

} // namespace ocs2 