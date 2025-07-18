#include "ocs2_ros_interfaces/command/DualArmTargetTrajectoriesInteractiveMarker.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <memory>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <utility>

namespace ocs2
{
    // Helper enum to distinguish between arms
    enum class ArmType { LEFT, RIGHT };

    DualArmTargetTrajectoriesInteractiveMarker::DualArmTargetTrajectoriesInteractiveMarker(
        rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
        DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories)
        : node_(std::move(node)),
          dualArmGoalPoseToTargetTrajectories_(std::move(dualArmGoalPoseToTargetTrajectories)),
          leftArmPosition_(0.0, 0.5, 1.0), // Default left arm position
          leftArmOrientation_(1.0, 0.0, 0.0, 0.0), // Default left arm orientation
          rightArmPosition_(0.0, -0.5, 1.0), // Default right arm position
          rightArmOrientation_(1.0, 0.0, 0.0, 0.0) // Default right arm orientation
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "simple_marker", node_);

        // observation subscriber
        auto observationCallback =
            [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg)
        {
            std::lock_guard lock(latestObservationMutex_);
            latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        };
        observationSubscriber_ =
            node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
                topicPrefix + "_mpc_observation", 1, observationCallback);

        // Trajectories publisher
        targetTrajectoriesPublisherPtr_ =
            std::make_unique<TargetTrajectoriesRosPublisher>(node_, topicPrefix);

        // Create separate menu handlers for left and right arms
        leftMenuHandler_ = std::make_unique<interactive_markers::MenuHandler>();
        rightMenuHandler_ = std::make_unique<interactive_markers::MenuHandler>();

        // create menu items for left arm
        auto leftArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processArmFeedback(feedback, ArmType::LEFT);
        };

        // create menu items for right arm
        auto rightArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processArmFeedback(feedback, ArmType::RIGHT);
        };

        // Common callback for sending both arms target
        auto sendBothArmsCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            // Get current poses from stored values (updated by feedback callbacks)
            const auto [leftPosition, leftOrientation] = getCurrentPose(ArmType::LEFT);
            const auto [rightPosition, rightOrientation] = getCurrentPose(ArmType::RIGHT);
            
            // Send both arms target
            SystemObservation observation;
            {
                std::lock_guard lock(latestObservationMutex_);
                observation = latestObservation_;
            }

            const auto targetTrajectories = dualArmGoalPoseToTargetTrajectories_(
                leftPosition, leftOrientation,
                rightPosition, rightOrientation,
                observation);

            targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
        };

        // Add menu items to left arm menu
        leftMenuHandler_->insert("Send left arm target", leftArmFeedbackCb);
        leftMenuHandler_->insert("Send both arms target", sendBothArmsCb);

        // Add menu items to right arm menu
        rightMenuHandler_->insert("Send right arm target", rightArmFeedbackCb);
        rightMenuHandler_->insert("Send both arms target", sendBothArmsCb);

        // create interactive markers for both arms
        auto leftArmMarker = createArmInteractiveMarker(ArmType::LEFT);
        auto rightArmMarker = createArmInteractiveMarker(ArmType::RIGHT);

        server_->insert(leftArmMarker);
        server_->insert(rightArmMarker);

        // Set up feedback callbacks for both markers to track their positions
        server_->setCallback(leftArmMarker.name, leftArmFeedbackCb);
        server_->setCallback(rightArmMarker.name, rightArmFeedbackCb);

        // Apply menu handlers to respective markers
        leftMenuHandler_->apply(*server_, leftArmMarker.name);
        rightMenuHandler_->apply(*server_, rightArmMarker.name);

        server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(), "Dual arm interactive markers are ready.");
    }

    visualization_msgs::msg::InteractiveMarker
    DualArmTargetTrajectoriesInteractiveMarker::createArmInteractiveMarker(ArmType armType) const
    {
        const bool isLeftArm = armType == ArmType::LEFT;
        
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = "world";
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = isLeftArm ? "LeftArmGoal" : "RightArmGoal";
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = (isLeftArm ? "Left" : "Right") + std::string(" arm target - Right click to send command");
        
        // Set position and orientation based on arm type
        const auto& position = isLeftArm ? leftArmPosition_ : rightArmPosition_;
        const auto& orientation = isLeftArm ? leftArmOrientation_ : rightArmOrientation_;
        
        interactiveMarker.pose.position.x = position.x();
        interactiveMarker.pose.position.y = position.y();
        interactiveMarker.pose.position.z = position.z();
        interactiveMarker.pose.orientation.w = orientation.w();
        interactiveMarker.pose.orientation.x = orientation.x();
        interactiveMarker.pose.orientation.y = orientation.y();
        interactiveMarker.pose.orientation.z = orientation.z();

        // create a colored box marker
        const auto boxMarker = createBoxMarker(armType);

        // create a non-interactive control which contains the box
        visualization_msgs::msg::InteractiveMarkerControl boxControl;
        boxControl.always_visible = true;
        boxControl.markers.push_back(boxMarker);
        boxControl.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

        // add the control to the interactive marker
        interactiveMarker.controls.push_back(boxControl);

        // add movement and rotation controls
        addMovementControls(interactiveMarker);

        return interactiveMarker;
    }

    visualization_msgs::msg::Marker
    DualArmTargetTrajectoriesInteractiveMarker::createBoxMarker(ArmType armType) const
    {
        const bool isLeftArm = (armType == ArmType::LEFT);
        
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        if (isLeftArm) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0; // Blue for left arm
        } else {
            marker.color.r = 1.0; // Red for right arm
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 0.7;
        
        return marker;
    }

    void DualArmTargetTrajectoriesInteractiveMarker::addMovementControls(
        visualization_msgs::msg::InteractiveMarker& interactiveMarker) const
    {
        // X-axis controls
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);

        // Z-axis controls
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);

        // Y-axis controls
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);
    }

    std::pair<Eigen::Vector3d, Eigen::Quaterniond>
    DualArmTargetTrajectoriesInteractiveMarker::getCurrentPose(ArmType armType) const
    {
        // Return the stored values which are kept up-to-date by feedback callbacks
        if (armType == ArmType::LEFT) {
            return {leftArmPosition_, leftArmOrientation_};
        }
        return {rightArmPosition_, rightArmOrientation_};
    }

    void DualArmTargetTrajectoriesInteractiveMarker::processArmFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
        ArmType armType)
    {
        const bool isLeftArm = (armType == ArmType::LEFT);
        
        // Update arm pose
        Eigen::Vector3d& position = isLeftArm ? leftArmPosition_ : rightArmPosition_;
        Eigen::Quaterniond& orientation = isLeftArm ? leftArmOrientation_ : rightArmOrientation_;
        
        position = Eigen::Vector3d(feedback->pose.position.x,
                                   feedback->pose.position.y,
                                   feedback->pose.position.z);
        orientation = Eigen::Quaterniond(feedback->pose.orientation.w,
                                         feedback->pose.orientation.x,
                                         feedback->pose.orientation.y,
                                         feedback->pose.orientation.z);

        // Only publish trajectories if this is a menu feedback (not just position update)
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
            // get the latest observation
            SystemObservation observation;
            {
                std::lock_guard lock(latestObservationMutex_);
                observation = latestObservation_;
            }

            // get TargetTrajectories for both arms
            const auto targetTrajectories = dualArmGoalPoseToTargetTrajectories_(
                leftArmPosition_, leftArmOrientation_,
                rightArmPosition_, rightArmOrientation_,
                observation);

            // publish TargetTrajectories
            targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
        }
    }
} // namespace ocs2
