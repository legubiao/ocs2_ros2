#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace ocs2 {
    /**
     * Unified interactive marker class that supports both single arm and dual arm modes.
     * This class combines the functionality of both TargetTrajectoriesInteractiveMarker 
     * and DualArmTargetTrajectoriesInteractiveMarker.
     */
    class UnifiedTargetTrajectoriesInteractiveMarker final {
    public:
        // Function types for different modes
        using SingleArmGoalPoseToTargetTrajectories = std::function<TargetTrajectories(
            const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
            const SystemObservation& observation)>;

        using DualArmGoalPoseToTargetTrajectories = std::function<TargetTrajectories(
            const Eigen::Vector3d& leftPosition, const Eigen::Quaterniond& leftOrientation,
            const Eigen::Vector3d& rightPosition, const Eigen::Quaterniond& rightOrientation,
            const SystemObservation& observation)>;

        /**
         * Constructor for single arm mode
         *
         * @param [in] node: ROS node handle.
         * @param [in] topicPrefix: The TargetTrajectories will be published on
         * "topicPrefix_mpc_target" topic. Moreover, the latest observation is be
         * expected on "topicPrefix_mpc_observation" topic.
         * @param [in] goalPoseToTargetTrajectories: A function which transforms the
         * commanded pose to TargetTrajectories.
         * @param [in] publishRate: Publishing rate for continuous mode (Hz), default 10Hz.
         */
        UnifiedTargetTrajectoriesInteractiveMarker(
            rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
            SingleArmGoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
            double publishRate = 10.0);

        /**
         * Constructor for dual arm mode
         *
         * @param [in] node: ROS node handle.
         * @param [in] topicPrefix: The TargetTrajectories will be published on
         * "topicPrefix_mpc_target" topic. Moreover, the latest observation is be
         * expected on "topicPrefix_mpc_observation" topic.
         * @param [in] dualArmGoalPoseToTargetTrajectories: A function which transforms the
         * commanded poses to TargetTrajectories.
         * @param [in] publishRate: Publishing rate for continuous mode (Hz), default 10Hz.
         */
        UnifiedTargetTrajectoriesInteractiveMarker(
            rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
            DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories,
            double publishRate = 10.0);

        /**
         * Spins ROS to update the interactive markers.
         */
        void publishInteractiveMarker() const { spin(node_); }

    private:
        // Mode enum
        enum class Mode { SINGLE_ARM, DUAL_ARM };

        // Helper enum for dual arm
        enum class ArmType { LEFT, RIGHT };

        // Core setup methods
        void setupCommon();
        void setupSingleArmMode();
        void setupDualArmMode();
        void setupObservationSubscriber();
        void setupTrajectoriesPublisher();
        void setupTimer();
        void setupJoystickSubscriber();

        // Marker creation methods
        visualization_msgs::msg::InteractiveMarker createSingleArmMarker() const;
        visualization_msgs::msg::InteractiveMarker createDualArmMarker(ArmType armType) const;
        visualization_msgs::msg::Marker createBoxMarker(const std::string& color = "grey") const;
        void addMovementControls(visualization_msgs::msg::InteractiveMarker& interactiveMarker) const;

        // Feedback processing methods
        void processSingleArmFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
        void processDualArmFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, ArmType armType);
        void processDualArmMenuFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

        // Menu handling methods
        void setupSingleArmMenu();
        void setupDualArmMenus();
        void updateSingleArmMenuVisibility();
        void updateDualArmMenuVisibility();

        // Trajectory sending methods
        void sendSingleArmTrajectories();
        void sendDualArmTrajectories();
        void togglePublishMode();
        void continuousPublishCallback();
        void joystickCallback(sensor_msgs::msg::Joy::SharedPtr msg);
        void updateMarkerShape();

        // Core members
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
        std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
        mutable std::mutex latestObservationMutex_;
        SystemObservation latestObservation_;

        // Mode and configuration
        Mode mode_;
        double publishRate_;
        bool continuousMode_;
        rclcpp::TimerBase::SharedPtr publishTimer_;
        std::string topicPrefix_;

        // Function objects
        std::function<TargetTrajectories(const Eigen::Vector3d&, const Eigen::Quaterniond&, const SystemObservation&)> singleArmFunction_;
        std::function<TargetTrajectories(const Eigen::Vector3d&, const Eigen::Quaterniond&, const Eigen::Vector3d&, const Eigen::Quaterniond&, const SystemObservation&)> dualArmFunction_;

        // Menu handlers
        std::shared_ptr<interactive_markers::MenuHandler> singleArmMenuHandler_;
        std::shared_ptr<interactive_markers::MenuHandler> leftArmMenuHandler_;
        std::shared_ptr<interactive_markers::MenuHandler> rightArmMenuHandler_;

        // Menu handles for dynamic visibility control (single arm)
        interactive_markers::MenuHandler::EntryHandle sendPoseHandle_;
        interactive_markers::MenuHandler::EntryHandle toggleModeHandle_;
        
        // Menu handles for dynamic visibility control (dual arm)
        interactive_markers::MenuHandler::EntryHandle leftArmSendHandle_;
        interactive_markers::MenuHandler::EntryHandle leftArmBothHandle_;
        interactive_markers::MenuHandler::EntryHandle leftArmToggleHandle_;
        interactive_markers::MenuHandler::EntryHandle rightArmSendHandle_;
        interactive_markers::MenuHandler::EntryHandle rightArmBothHandle_;
        interactive_markers::MenuHandler::EntryHandle rightArmToggleHandle_;

        // Position storage
        mutable std::mutex markerPoseMutex_;
        Eigen::Vector3d singleArmPosition_;
        Eigen::Quaterniond singleArmOrientation_;
        Eigen::Vector3d leftArmPosition_;
        Eigen::Quaterniond leftArmOrientation_;
        Eigen::Vector3d rightArmPosition_;
        Eigen::Quaterniond rightArmOrientation_;
        
        // Joystick control for both single and dual arm modes
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSubscriber_;
        bool joystickEnabled_;
        double joystickLinearScale_;
        double joystickAngularScale_;
        Eigen::Vector3d joystickPosition_;
        Eigen::Quaterniond joystickOrientation_;
        ArmType activeArm_;  // 当前激活的手臂（双臂模式）
        
        // Button cooldown control (shared by all buttons)
        bool anyButtonPressed_;
        rclcpp::Time lastButtonTime_;
        double buttonCooldownDuration_;
        
        // Joystick update rate control
        rclcpp::Time lastJoystickUpdateTime_;
        double joystickUpdateRate_;
    };
} // namespace ocs2 