#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    /**
     * Marker auto position wrapper that automatically updates marker positions based on end effector poses.
     * This class provides automatic marker position synchronization with robot end effector positions.
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
         */
        MarkerAutoPositionWrapper(
            rclcpp::Node::SharedPtr node,
            const std::string& topicPrefix,
            IMarkerControl* markerControl,
            UpdateMode updateMode = UpdateMode::INITIALIZATION,
            double cooldownDuration = 3.0,
            double maxUpdateFrequency = 1.0);

        /**
         * Destructor
         */
        ~MarkerAutoPositionWrapper() = default;



        /**
         * Set update mode
         * @param mode Update mode
         */
        void setUpdateMode(UpdateMode mode);

        /**
         * Set cooldown duration
         * @param duration Cooldown duration (seconds)
         */
        void setCooldownDuration(double duration);

        /**
         * Set maximum update frequency
         * @param frequency Maximum update frequency (Hz)
         */
        void setMaxUpdateFrequency(double frequency);

        /**
         * Reset cooldown
         */
        void resetCooldown();

        /**
         * Get current update mode
         * @return Current update mode
         */
        UpdateMode getUpdateMode() const { return updateMode_; }

        /**
         * Get cooldown duration
         * @return Cooldown duration (seconds)
         */
        double getCooldownDuration() const { return cooldownDuration_; }

    private:
        /**
         * End effector pose callback
         * @param msg Pose message
         */
        void endEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

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
         * Update marker position
         * @param msg Pose message
         */
        void updateMarkerPosition(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        /**
         * Check if position should be updated
         * @return true if should update, false otherwise
         */
        bool shouldUpdatePosition() const;

        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr endEffectorPoseSubscriber_;
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


        bool initialized_;
        bool updateEnabled_;
        rclcpp::Time lastMpcObservationTime_;
        rclcpp::Time lastEndEffectorPoseTime_;
        rclcpp::Time lastUpdateTime_;  // Last update time

        // Thread safety
        mutable std::mutex stateMutex_;
    };

} // namespace ocs2 