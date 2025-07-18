#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace ocs2 {
    /**
     * This class lets the user to command dual arm robot from interactive markers.
     */
    class DualArmTargetTrajectoriesInteractiveMarker final {
    public:
        using DualArmGoalPoseToTargetTrajectories = std::function<TargetTrajectories(
            const Eigen::Vector3d& leftPosition, const Eigen::Quaterniond& leftOrientation,
            const Eigen::Vector3d& rightPosition, const Eigen::Quaterniond& rightOrientation,
            const SystemObservation& observation)>;

        /**
         * Constructor
         *
         * @param [in] node: ROS node handle.
         * @param [in] topicPrefix: The TargetTrajectories will be published on
         * "topicPrefix_mpc_target" topic. Moreover, the latest observation is be
         * expected on "topicPrefix_mpc_observation" topic.
         * @param [in] dualArmGoalPoseToTargetTrajectories: A function which transforms the
         * commanded poses to TargetTrajectories.
         */
        DualArmTargetTrajectoriesInteractiveMarker(
            rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
            DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories);

        /**
         * Spins ROS to update the interactive markers.
         */
        void publishInteractiveMarker() const { spin(node_); }

    private:
        // Helper enum to distinguish between arms
        enum class ArmType { LEFT, RIGHT };

        // Unified methods to reduce code duplication
        visualization_msgs::msg::InteractiveMarker createArmInteractiveMarker(ArmType armType) const;
        visualization_msgs::msg::Marker createBoxMarker(ArmType armType) const;
        void addMovementControls(visualization_msgs::msg::InteractiveMarker& interactiveMarker) const;
        void processArmFeedback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
            ArmType armType);
        
        // Helper method to get current poses from interactive markers
        std::pair<Eigen::Vector3d, Eigen::Quaterniond> getCurrentPose(ArmType armType) const;

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<interactive_markers::MenuHandler> leftMenuHandler_;
        std::shared_ptr<interactive_markers::MenuHandler> rightMenuHandler_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories_;

        std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;
        mutable std::mutex latestObservationMutex_;
        SystemObservation latestObservation_;

        // Store current poses for both arms
        Eigen::Vector3d leftArmPosition_;
        Eigen::Quaterniond leftArmOrientation_;
        Eigen::Vector3d rightArmPosition_;
        Eigen::Quaterniond rightArmOrientation_;
    };
} // namespace ocs2 