#include "ocs2_ros_interfaces/command/DualArmTargetTrajectoriesInteractiveMarker.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <memory>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <utility>

namespace ocs2
{
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
            "dual_arm_marker", node_);

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

        menuHandler_ = std::make_unique<interactive_markers::MenuHandler>();

        // create menu items
        auto leftArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processLeftArmFeedback(feedback);
        };
        auto rightArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processRightArmFeedback(feedback);
        };

        menuHandler_->insert("Send left arm target", leftArmFeedbackCb);
        menuHandler_->insert("Send right arm target", rightArmFeedbackCb);
        menuHandler_->insert("Send both arms target",
                             [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                             {
                                 // Send both arms target
                                 SystemObservation observation;
                                 {
                                     std::lock_guard<std::mutex> lock(latestObservationMutex_);
                                     observation = latestObservation_;
                                 }

                                 const auto targetTrajectories = dualArmGoalPoseToTargetTrajectories_(
                                     leftArmPosition_, leftArmOrientation_,
                                     rightArmPosition_, rightArmOrientation_,
                                     observation);

                                 targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
                             });

        // create interactive markers for both arms
        auto leftArmMarker = createLeftArmInteractiveMarker();
        auto rightArmMarker = createRightArmInteractiveMarker();

        server_->insert(leftArmMarker);
        server_->insert(rightArmMarker);

        menuHandler_->apply(*server_, leftArmMarker.name);
        menuHandler_->apply(*server_, rightArmMarker.name);

        server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(), "Dual arm interactive markers are ready.");
    }

    visualization_msgs::msg::InteractiveMarker
    DualArmTargetTrajectoriesInteractiveMarker::createLeftArmInteractiveMarker() const
    {
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = "world";
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = "LeftArmGoal";
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = "Left arm target - Right click to send command";
        interactiveMarker.pose.position.x = leftArmPosition_.x();
        interactiveMarker.pose.position.y = leftArmPosition_.y();
        interactiveMarker.pose.position.z = leftArmPosition_.z();
        interactiveMarker.pose.orientation.w = leftArmOrientation_.w();
        interactiveMarker.pose.orientation.x = leftArmOrientation_.x();
        interactiveMarker.pose.orientation.y = leftArmOrientation_.y();
        interactiveMarker.pose.orientation.z = leftArmOrientation_.z();

        // create a blue box marker for left arm
        const auto boxMarker = []()
        {
            visualization_msgs::msg::Marker marker;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0; // Blue for left arm
            marker.color.a = 0.7;
            return marker;
        }();

        // create a non-interactive control which contains the box
        visualization_msgs::msg::InteractiveMarkerControl boxControl;
        boxControl.always_visible = true;
        boxControl.markers.push_back(boxMarker);
        boxControl.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

        // add the control to the interactive marker
        interactiveMarker.controls.push_back(boxControl);

        // create controls for movement and rotation
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

        return interactiveMarker;
    }

    visualization_msgs::msg::InteractiveMarker
    DualArmTargetTrajectoriesInteractiveMarker::createRightArmInteractiveMarker() const
    {
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = "world";
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = "RightArmGoal";
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = "Right arm target - Right click to send command";
        interactiveMarker.pose.position.x = rightArmPosition_.x();
        interactiveMarker.pose.position.y = rightArmPosition_.y();
        interactiveMarker.pose.position.z = rightArmPosition_.z();
        interactiveMarker.pose.orientation.w = rightArmOrientation_.w();
        interactiveMarker.pose.orientation.x = rightArmOrientation_.x();
        interactiveMarker.pose.orientation.y = rightArmOrientation_.y();
        interactiveMarker.pose.orientation.z = rightArmOrientation_.z();

        // create a red box marker for right arm
        const auto boxMarker = []()
        {
            visualization_msgs::msg::Marker marker;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0; // Red for right arm
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
            return marker;
        }();

        // create a non-interactive control which contains the box
        visualization_msgs::msg::InteractiveMarkerControl boxControl;
        boxControl.always_visible = true;
        boxControl.markers.push_back(boxMarker);
        boxControl.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

        // add the control to the interactive marker
        interactiveMarker.controls.push_back(boxControl);

        // create controls for movement and rotation
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

        return interactiveMarker;
    }

    void DualArmTargetTrajectoriesInteractiveMarker::processLeftArmFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        // Update left arm pose
        leftArmPosition_ = Eigen::Vector3d(feedback->pose.position.x,
                                           feedback->pose.position.y,
                                           feedback->pose.position.z);
        leftArmOrientation_ = Eigen::Quaterniond(feedback->pose.orientation.w,
                                                 feedback->pose.orientation.x,
                                                 feedback->pose.orientation.y,
                                                 feedback->pose.orientation.z);

        // get the latest observation
        SystemObservation observation;
        {
            std::lock_guard<std::mutex> lock(latestObservationMutex_);
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

    void DualArmTargetTrajectoriesInteractiveMarker::processRightArmFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        // Update right arm pose
        rightArmPosition_ = Eigen::Vector3d(feedback->pose.position.x,
                                            feedback->pose.position.y,
                                            feedback->pose.position.z);
        rightArmOrientation_ = Eigen::Quaterniond(feedback->pose.orientation.w,
                                                  feedback->pose.orientation.x,
                                                  feedback->pose.orientation.y,
                                                  feedback->pose.orientation.z);

        // get the latest observation
        SystemObservation observation;
        {
            std::lock_guard<std::mutex> lock(latestObservationMutex_);
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
} // namespace ocs2
