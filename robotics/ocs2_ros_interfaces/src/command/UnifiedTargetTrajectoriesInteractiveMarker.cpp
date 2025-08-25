#include "ocs2_ros_interfaces/command/UnifiedTargetTrajectoriesInteractiveMarker.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <memory>
#include <mutex>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <utility>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ocs2
{
    // Single arm constructor
    UnifiedTargetTrajectoriesInteractiveMarker::UnifiedTargetTrajectoriesInteractiveMarker(
        rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
        SingleArmGoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
        const double publishRate, std::string  frameId)
        : node_(std::move(node)),
          mode_(Mode::SINGLE_ARM),
          publishRate_(publishRate),
          continuousMode_(false),
          frameId_(std::move(frameId)),
          singleArmFunction_(std::move(goalPoseToTargetTrajectories)),
          singleArmPosition_(0.0, 0.0, 1.0),
          singleArmOrientation_(1.0, 0.0, 0.0, 0.0), // Default active arm
          activeArm_(ArmType::LEFT)
    {
        topicPrefix_ = topicPrefix;
        setupCommon();
        setupSingleArmMode();
    }

    // Dual arm constructor
    UnifiedTargetTrajectoriesInteractiveMarker::UnifiedTargetTrajectoriesInteractiveMarker(
        rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
        DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories,
        const double publishRate, std::string  frameId)
        : node_(std::move(node)),
          mode_(Mode::DUAL_ARM),
          publishRate_(publishRate),
          continuousMode_(false),
          frameId_(std::move(frameId)),
          dualArmFunction_(std::move(dualArmGoalPoseToTargetTrajectories)),
          leftArmPosition_(0.0, 0.5, 1.0),
          leftArmOrientation_(1.0, 0.0, 0.0, 0.0),
          rightArmPosition_(0.0, -0.5, 1.0),
          rightArmOrientation_(1.0, 0.0, 0.0, 0.0), // Default active arm
          activeArm_(ArmType::LEFT)
    {
        topicPrefix_ = topicPrefix;
        setupCommon();
        setupDualArmMode();
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupCommon()
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "simple_marker", node_);
        setupObservationSubscriber();
        setupTrajectoriesPublisher();
        setupTimer();
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupSingleArmMode()
    {
        setupSingleArmMenu();

        auto const interactiveMarker = createSingleArmMarker();
        server_->insert(interactiveMarker);

        // Set up feedback callback to track marker position in real-time
        auto feedbackCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processSingleArmFeedback(feedback);
        };
        server_->setCallback(interactiveMarker.name, feedbackCallback);

        singleArmMenuHandler_->apply(*server_, interactiveMarker.name);
        updateSingleArmMenuVisibility();

        server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(),
                    "Single arm interactive marker is ready. Right click to send command or toggle continuous mode.");
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupDualArmMode()
    {
        setupDualArmMenus();

        auto leftArmMarker = createDualArmMarker(ArmType::LEFT);
        auto rightArmMarker = createDualArmMarker(ArmType::RIGHT);

        server_->insert(leftArmMarker);
        server_->insert(rightArmMarker);

        // Set up feedback callbacks for both markers
        auto leftArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processDualArmFeedback(feedback, ArmType::LEFT);
        };
        auto rightArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processDualArmFeedback(feedback, ArmType::RIGHT);
        };

        server_->setCallback(leftArmMarker.name, leftArmFeedbackCb);
        server_->setCallback(rightArmMarker.name, rightArmFeedbackCb);

        leftArmMenuHandler_->apply(*server_, leftArmMarker.name);
        rightArmMenuHandler_->apply(*server_, rightArmMarker.name);

        updateDualArmMenuVisibility();

        server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(), "Dual arm interactive markers are ready.");
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupObservationSubscriber()
    {
        auto observationCallback = [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg)
        {
            std::lock_guard lock(latestObservationMutex_);
            latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        };
        observationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
            topicPrefix_ + "_mpc_observation", 1, observationCallback);
    }


    void UnifiedTargetTrajectoriesInteractiveMarker::setupTrajectoriesPublisher()
    {
        targetTrajectoriesPublisherPtr_ = std::make_unique<TargetTrajectoriesRosPublisher>(node_, topicPrefix_);
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupTimer()
    {
        publishTimer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0 / publishRate_),
            std::bind(&UnifiedTargetTrajectoriesInteractiveMarker::continuousPublishCallback, this));
    }


    visualization_msgs::msg::InteractiveMarker
    UnifiedTargetTrajectoriesInteractiveMarker::createSingleArmMarker() const
    {
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = frameId_;
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = "Goal";
        interactiveMarker.scale = 0.2;

        // Set position and orientation
        interactiveMarker.pose.position.x = singleArmPosition_.x();
        interactiveMarker.pose.position.y = singleArmPosition_.y();
        interactiveMarker.pose.position.z = singleArmPosition_.z();
        interactiveMarker.pose.orientation.w = singleArmOrientation_.w();
        interactiveMarker.pose.orientation.x = singleArmOrientation_.x();
        interactiveMarker.pose.orientation.y = singleArmOrientation_.y();
        interactiveMarker.pose.orientation.z = singleArmOrientation_.z();

        // Create a box marker
        const auto boxMarker = createBoxMarker();

        // Create a non-interactive control which contains the box
        visualization_msgs::msg::InteractiveMarkerControl boxControl;
        boxControl.always_visible = true;
        boxControl.markers.push_back(boxMarker);
        boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

        // Add the control to the interactive marker
        interactiveMarker.controls.push_back(boxControl);

        // Add movement controls
        addMovementControls(interactiveMarker);

        return interactiveMarker;
    }

    visualization_msgs::msg::InteractiveMarker
    UnifiedTargetTrajectoriesInteractiveMarker::createDualArmMarker(ArmType armType) const
    {
        const bool isLeftArm = armType == ArmType::LEFT;

        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = frameId_;
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = isLeftArm ? "LeftArmGoal" : "RightArmGoal";
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = isLeftArm ? "Left" : "Right";

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

        // Create a colored box marker
        const auto boxMarker = createBoxMarker(isLeftArm ? "blue" : "red");

        // Create a non-interactive control which contains the box
        visualization_msgs::msg::InteractiveMarkerControl boxControl;
        boxControl.always_visible = true;
        boxControl.markers.push_back(boxMarker);
        boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

        // Add the control to the interactive marker
        interactiveMarker.controls.push_back(boxControl);

        // Add movement controls
        addMovementControls(interactiveMarker);

        return interactiveMarker;
    }

    visualization_msgs::msg::Marker
    UnifiedTargetTrajectoriesInteractiveMarker::createBoxMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;

        // Use sphere in continuous mode, otherwise use cube
        if (continuousMode_)
        {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
        }
        else
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        if (color == "blue")
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if (color == "red")
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            // Default grey
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        marker.color.a = 0.7;

        return marker;
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::addMovementControls(
        visualization_msgs::msg::InteractiveMarker& interactiveMarker) const
    {
        // X-axis controls
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);

        // Z-axis controls
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);

        // Y-axis controls
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::processSingleArmFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        // Update current marker pose in real-time
        {
            std::lock_guard lock(markerPoseMutex_);
            singleArmPosition_ = Eigen::Vector3d(feedback->pose.position.x,
                                                 feedback->pose.position.y,
                                                 feedback->pose.position.z);
            singleArmOrientation_ = Eigen::Quaterniond(
                feedback->pose.orientation.w, feedback->pose.orientation.x,
                feedback->pose.orientation.y, feedback->pose.orientation.z);
        }

        // Only send trajectories if this is a menu feedback and not in continuous mode
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT && !continuousMode_)
        {
            sendSingleArmTrajectories();
        }
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::processDualArmFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
        ArmType armType)
    {
        const bool isLeftArm = armType == ArmType::LEFT;

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
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT)
        {
            sendDualArmTrajectories();
        }
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupSingleArmMenu()
    {
        singleArmMenuHandler_ = std::make_unique<interactive_markers::MenuHandler>();

        // Create menu items for mode switching
        auto sendPoseCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            // Use stored current position for menu actions
            std::lock_guard lock(markerPoseMutex_);
            sendSingleArmTrajectories();
        };

        auto toggleModeCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            togglePublishMode();
        };

        // Store menu item handles for later manipulation
        sendPoseHandle_ = singleArmMenuHandler_->insert("Send target pose", sendPoseCallback);
        toggleModeHandle_ = singleArmMenuHandler_->insert("Toggle continuous mode", toggleModeCallback);
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setupDualArmMenus()
    {
        leftArmMenuHandler_ = std::make_unique<interactive_markers::MenuHandler>();
        rightArmMenuHandler_ = std::make_unique<interactive_markers::MenuHandler>();

        // create menu items for left arm
        auto leftArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processDualArmFeedback(feedback, ArmType::LEFT);
        };

        // create menu items for right arm
        auto rightArmFeedbackCb = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            processDualArmFeedback(feedback, ArmType::RIGHT);
        };

        // Common callback for sending both arms target
        auto sendBothArmsCb = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            sendDualArmTrajectories();
        };

        // Toggle continuous mode callback
        auto toggleModeCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            togglePublishMode();
        };

        // Add menu items to left arm menu
        leftArmSendHandle_ = leftArmMenuHandler_->insert("Send left arm target", leftArmFeedbackCb);
        leftArmBothHandle_ = leftArmMenuHandler_->insert("Send both arms target", sendBothArmsCb);
        leftArmToggleHandle_ = leftArmMenuHandler_->insert("Toggle continuous mode", toggleModeCallback);

        // Add menu items to right arm menu
        rightArmSendHandle_ = rightArmMenuHandler_->insert("Send right arm target", rightArmFeedbackCb);
        rightArmBothHandle_ = rightArmMenuHandler_->insert("Send both arms target", sendBothArmsCb);
        rightArmToggleHandle_ = rightArmMenuHandler_->insert("Toggle continuous mode", toggleModeCallback);
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::updateSingleArmMenuVisibility()
    {
        if (continuousMode_)
        {
            // Hide "Send target pose" button in continuous mode
            singleArmMenuHandler_->setVisible(sendPoseHandle_, false);
        }
        else
        {
            // Show "Send target pose" button in manual mode
            singleArmMenuHandler_->setVisible(sendPoseHandle_, true);
        }
        singleArmMenuHandler_->reApply(*server_);
        server_->applyChanges();
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::updateDualArmMenuVisibility()
    {
        if (continuousMode_)
        {
            // Hide send buttons in continuous mode
            leftArmMenuHandler_->setVisible(leftArmSendHandle_, false);
            leftArmMenuHandler_->setVisible(leftArmBothHandle_, false);
            rightArmMenuHandler_->setVisible(rightArmSendHandle_, false);
            rightArmMenuHandler_->setVisible(rightArmBothHandle_, false);
        }
        else
        {
            // Show send buttons in manual mode
            leftArmMenuHandler_->setVisible(leftArmSendHandle_, true);
            leftArmMenuHandler_->setVisible(leftArmBothHandle_, true);
            rightArmMenuHandler_->setVisible(rightArmSendHandle_, true);
            rightArmMenuHandler_->setVisible(rightArmBothHandle_, true);
        }

        leftArmMenuHandler_->reApply(*server_);
        rightArmMenuHandler_->reApply(*server_);
        server_->applyChanges();
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::sendSingleArmTrajectories()
    {
        SystemObservation observation;
        {
            std::lock_guard lock(latestObservationMutex_);
            observation = latestObservation_;
        }

        const auto targetTrajectories = singleArmFunction_(singleArmPosition_, singleArmOrientation_, observation);
        targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::sendDualArmTrajectories()
    {
        SystemObservation observation;
        {
            std::lock_guard lock(latestObservationMutex_);
            observation = latestObservation_;
        }

        const auto targetTrajectories = dualArmFunction_(
            leftArmPosition_, leftArmOrientation_,
            rightArmPosition_, rightArmOrientation_,
            observation);

        targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::togglePublishMode()
    {
        continuousMode_ = !continuousMode_;
        if (continuousMode_)
        {
            RCLCPP_INFO(node_->get_logger(), "Continuous mode enabled. Markers will publish trajectories at %f Hz.",
                        publishRate_);
            RCLCPP_INFO(node_->get_logger(), "🎯 Marker changed to SPHERE shape for continuous mode.");
        }
        else
        {
            if (mode_ == Mode::SINGLE_ARM)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Continuous mode disabled. Click 'Send target pose' to send trajectories.");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "Continuous mode disabled. Click 'Send target' to send trajectories.");
            }
            RCLCPP_INFO(node_->get_logger(), "🎯 Marker changed to CUBE shape for manual mode.");
        }

        // Update marker display
        updateMarkerShape();

        if (mode_ == Mode::SINGLE_ARM)
        {
            updateSingleArmMenuVisibility();
        }
        else
        {
            updateDualArmMenuVisibility();
        }
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::continuousPublishCallback()
    {
        if (continuousMode_)
        {
            if (mode_ == Mode::SINGLE_ARM)
            {
                sendSingleArmTrajectories();
            }
            else
            {
                sendDualArmTrajectories();
            }
        }
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::updateMarkerShape()
    {
        if (mode_ == Mode::SINGLE_ARM)
        {
            // Recreate single arm marker
            auto marker = createSingleArmMarker();
            server_->insert(
                marker, [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                {
                    processSingleArmFeedback(feedback);
                });
            singleArmMenuHandler_->apply(*server_, marker.name);
        }
        else
        {
            // Recreate dual arm markers
            auto leftArmMarker = createDualArmMarker(ArmType::LEFT);
            auto rightArmMarker = createDualArmMarker(ArmType::RIGHT);

            server_->insert(leftArmMarker,
                            [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                            {
                                processDualArmFeedback(feedback, ArmType::LEFT);
                            });
            server_->insert(rightArmMarker,
                            [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                            {
                                processDualArmFeedback(feedback, ArmType::RIGHT);
                            });

            leftArmMenuHandler_->apply(*server_, leftArmMarker.name);
            rightArmMenuHandler_->apply(*server_, rightArmMarker.name);
        }

        server_->applyChanges();
    }


    // IMarkerControl interface implementation
    void UnifiedTargetTrajectoriesInteractiveMarker::setSingleArmPose(const Eigen::Vector3d& position,
                                                                      const Eigen::Quaterniond& orientation)
    {
        std::lock_guard lock(markerPoseMutex_);
        singleArmPosition_ = position;
        singleArmOrientation_ = orientation;
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setDualArmPose(ArmType armType, const Eigen::Vector3d& position,
                                                                    const Eigen::Quaterniond& orientation)
    {
        std::lock_guard lock(markerPoseMutex_);
        if (armType == ArmType::LEFT)
        {
            leftArmPosition_ = position;
            leftArmOrientation_ = orientation;
        }
        else
        {
            rightArmPosition_ = position;
            rightArmOrientation_ = orientation;
        }
    }

    std::pair<Eigen::Vector3d, Eigen::Quaterniond> UnifiedTargetTrajectoriesInteractiveMarker::getSingleArmPose() const
    {
        std::lock_guard lock(markerPoseMutex_);
        return {singleArmPosition_, singleArmOrientation_};
    }

    std::pair<Eigen::Vector3d, Eigen::Quaterniond> UnifiedTargetTrajectoriesInteractiveMarker::getDualArmPose(
        ArmType armType) const
    {
        std::lock_guard lock(markerPoseMutex_);
        if (armType == ArmType::LEFT)
        {
            return {leftArmPosition_, leftArmOrientation_};
        }
        return {rightArmPosition_, rightArmOrientation_};
    }

    bool UnifiedTargetTrajectoriesInteractiveMarker::isContinuousMode() const
    {
        return continuousMode_;
    }

    IMarkerControl::Mode UnifiedTargetTrajectoriesInteractiveMarker::getMode() const
    {
        return mode_;
    }

    IMarkerControl::ArmType UnifiedTargetTrajectoriesInteractiveMarker::getActiveArm() const
    {
        return activeArm_;
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::setActiveArm(ArmType armType)
    {
        activeArm_ = armType;
    }

    void UnifiedTargetTrajectoriesInteractiveMarker::updateMarkerDisplay(
        const std::string& markerName, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        geometry_msgs::msg::Pose markerPose;
        markerPose.position.x = position.x();
        markerPose.position.y = position.y();
        markerPose.position.z = position.z();
        markerPose.orientation.w = orientation.w();
        markerPose.orientation.x = orientation.x();
        markerPose.orientation.y = orientation.y();
        markerPose.orientation.z = orientation.z();

        server_->setPose(markerName, markerPose);
        server_->applyChanges();
    }

    UnifiedTargetTrajectoriesInteractiveMarker::~UnifiedTargetTrajectoriesInteractiveMarker()
    {
        // Clean up timer
        if (publishTimer_)
        {
            publishTimer_->cancel();
        }
    }
} // namespace ocs2
