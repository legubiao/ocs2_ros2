#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    /**
     * Joystick marker wrapper that handles joystick input and controls marker operations.
     * This class acts as a wrapper/adapter between joystick input and marker control.
     * It is decoupled from specific marker implementations through the IMarkerControl interface.
     */
    class JoystickMarkerWrapper {
    public:
        /**
         * Constructor
         * @param node ROS node handle
         * @param markerControl Pointer to marker control interface
         * @param linearScale Scale factor for linear movement
         * @param angularScale Scale factor for angular movement
         * @param updateRate Update rate for joystick processing (Hz)
         */
        JoystickMarkerWrapper(
            rclcpp::Node::SharedPtr node,
            IMarkerControl* markerControl,
            double linearScale = 0.01,
            double angularScale = 0.01,
            double updateRate = 20.0);

        /**
         * Destructor
         */
        ~JoystickMarkerWrapper() = default;

        /**
         * Enable joystick control
         */
        void enable();

        /**
         * Disable joystick control
         */
        void disable();

        /**
         * Check if joystick control is enabled
         * @return true if enabled, false otherwise
         */
        bool isEnabled() const { return enabled_.load(); }

        /**
         * Set linear movement scale
         * @param scale Scale factor
         */
        void setLinearScale(double scale) { linearScale_ = scale; }

        /**
         * Set angular movement scale
         * @param scale Scale factor
         */
        void setAngularScale(double scale) { angularScale_ = scale; }

        /**
         * Set update rate
         * @param rate Update rate in Hz
         */
        void setUpdateRate(double rate) { updateRate_ = rate; }

    private:
        /**
         * Joystick callback function
         * @param msg Joy message
         */
        void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * Process button inputs
         * @param msg Joy message
         */
        void processButtons(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * Process axis inputs for position and orientation control
         * @param msg Joy message
         */
        void processAxes(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * Update marker position based on joystick input
         * @param position New position
         * @param orientation New orientation
         */
        void updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSubscriber_;

        // Marker control interface
        IMarkerControl* markerControl_;

        // Control parameters
        double linearScale_;
        double angularScale_;
        double updateRate_;

        // State management
        std::atomic<bool> enabled_;
        std::mutex stateMutex_;

        // Timing control
        rclcpp::Time lastUpdateTime_;
        rclcpp::Time lastButtonTime_;
        double buttonCooldownDuration_;

        // Button state tracking
        bool anyButtonPressed_;
        bool lastButtonStates_[6]; // Track last state of buttons 0-5

        // Current joystick position and orientation
        Eigen::Vector3d currentPosition_;
        Eigen::Quaterniond currentOrientation_;
    };

} // namespace ocs2 