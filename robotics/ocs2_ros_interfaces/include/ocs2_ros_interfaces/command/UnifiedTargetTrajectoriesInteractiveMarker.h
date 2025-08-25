#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/command/IMarkerControl.h>
#include <rclcpp/rclcpp.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ocs2
{
    /**
     * Unified interactive marker class that supports both single arm and dual arm modes.
     * This class combines the functionality of both TargetTrajectoriesInteractiveMarker 
     * and DualArmTargetTrajectoriesInteractiveMarker.
     */
    class UnifiedTargetTrajectoriesInteractiveMarker final : public IMarkerControl
    {
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
         * @param [in] frameId: Frame ID for the interactive marker, default "world".
         */
        UnifiedTargetTrajectoriesInteractiveMarker(
            rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
            SingleArmGoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
            double publishRate = 10.0,
            std::string frameId = "world");

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
         * @param [in] frameId: Frame ID for the interactive marker, default "world".
         */
        UnifiedTargetTrajectoriesInteractiveMarker(
            rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
            DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories,
            double publishRate = 10.0,
            std::string frameId = "world");

        ~UnifiedTargetTrajectoriesInteractiveMarker() override;

        // Public methods

        // IMarkerControl interface implementation
        void setSingleArmPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) override;
        void setDualArmPose(ArmType armType, const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation) override;
        std::pair<Eigen::Vector3d, Eigen::Quaterniond> getSingleArmPose() const override;
        std::pair<Eigen::Vector3d, Eigen::Quaterniond> getDualArmPose(ArmType armType) const override;
        void sendSingleArmTrajectories() override;
        void sendDualArmTrajectories() override;
        void togglePublishMode() override;
        bool isContinuousMode() const override;
        Mode getMode() const override;
        ArmType getActiveArm() const override;
        void setActiveArm(ArmType armType) override;
        void updateMarkerDisplay(const std::string& markerName, const Eigen::Vector3d& position,
                                 const Eigen::Quaterniond& orientation) override;

    private:
        // Core setup methods
        void setupCommon();
        void setupSingleArmMode();
        void setupDualArmMode();
        void setupObservationSubscriber();
        void setupTrajectoriesPublisher();
        void setupTimer();

        // Marker creation methods
        visualization_msgs::msg::InteractiveMarker createSingleArmMarker() const;
        visualization_msgs::msg::InteractiveMarker createDualArmMarker(ArmType armType) const;
        visualization_msgs::msg::Marker createBoxMarker(const std::string& color = "grey") const;
        void addMovementControls(visualization_msgs::msg::InteractiveMarker& interactiveMarker) const;

        // Feedback processing methods
        void processSingleArmFeedback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
        void processDualArmFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
                                    ArmType armType);

        // Menu handling methods
        void setupSingleArmMenu();
        void setupDualArmMenus();
        void updateSingleArmMenuVisibility();
        void updateDualArmMenuVisibility();

        // Timer and callback methods
        void continuousPublishCallback();
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
        // Timer and publishing
        rclcpp::TimerBase::SharedPtr publishTimer_;
        std::string topicPrefix_;

        // Frame information
        std::string frameId_;

        // Function objects
        std::function<TargetTrajectories(const Eigen::Vector3d&, const Eigen::Quaterniond&, const SystemObservation&)>
        singleArmFunction_;
        std::function<TargetTrajectories(const Eigen::Vector3d&, const Eigen::Quaterniond&, const Eigen::Vector3d&,
                                         const Eigen::Quaterniond&, const SystemObservation&)> dualArmFunction_;

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

        // Active arm for dual arm mode
        ArmType activeArm_; // Currently active arm (dual-arm mode)

        // Marker initialization
    };
} // namespace ocs2
