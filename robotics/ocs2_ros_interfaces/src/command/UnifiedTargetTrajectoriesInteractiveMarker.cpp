#include "ocs2_ros_interfaces/command/UnifiedTargetTrajectoriesInteractiveMarker.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <memory>
#include <ocs2_msgs/msg/mpc_observation.hpp>

namespace ocs2
{
    // Single arm constructor
    UnifiedTargetTrajectoriesInteractiveMarker::UnifiedTargetTrajectoriesInteractiveMarker(
        rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
        SingleArmGoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
        double publishRate)
        : node_(std::move(node)),
          mode_(Mode::SINGLE_ARM),
          publishRate_(publishRate),
          continuousMode_(false),
          singleArmFunction_(std::move(goalPoseToTargetTrajectories)),
          singleArmPosition_(0.0, 0.0, 1.0),
          singleArmOrientation_(1.0, 0.0, 0.0, 0.0),
          joystickEnabled_(false),
          joystickLinearScale_(0.01),
          joystickAngularScale_(0.01),
          joystickPosition_(0.0, 0.0, 1.0),
          joystickOrientation_(1.0, 0.0, 0.0, 0.0),
          activeArm_(ArmType::RIGHT), // Default active arm
          anyButtonPressed_(false),
          lastButtonTime_(node_->now()),
          buttonCooldownDuration_(0.5), // 0.5 second cooldown
          lastJoystickUpdateTime_(node_->now()),
          joystickUpdateRate_(20.0)
    {
        // 20Hz update rate

        topicPrefix_ = topicPrefix;
        setupCommon();
        setupSingleArmMode();
        setupJoystickSubscriber();
    }

    // Dual arm constructor
    UnifiedTargetTrajectoriesInteractiveMarker::UnifiedTargetTrajectoriesInteractiveMarker(
        rclcpp::Node::SharedPtr node, const std::string& topicPrefix,
        DualArmGoalPoseToTargetTrajectories dualArmGoalPoseToTargetTrajectories,
        double publishRate)
        : node_(std::move(node)),
          mode_(Mode::DUAL_ARM),
          publishRate_(publishRate),
          continuousMode_(false),
          dualArmFunction_(std::move(dualArmGoalPoseToTargetTrajectories)),
          leftArmPosition_(0.0, 0.5, 1.0),
          leftArmOrientation_(1.0, 0.0, 0.0, 0.0),
          rightArmPosition_(0.0, -0.5, 1.0),
          rightArmOrientation_(1.0, 0.0, 0.0, 0.0),
          joystickEnabled_(false),
          joystickLinearScale_(0.01),
          joystickAngularScale_(0.01),
          joystickPosition_(0.0, 0.0, 1.0),
          joystickOrientation_(1.0, 0.0, 0.0, 0.0),
          activeArm_(ArmType::RIGHT), // Default active arm
          anyButtonPressed_(false),
          lastButtonTime_(node_->now()),
          buttonCooldownDuration_(0.5), // 0.5 second cooldown
          lastJoystickUpdateTime_(node_->now()),
          joystickUpdateRate_(20.0)
    {
        // 20Hz update rate

        topicPrefix_ = topicPrefix;
        setupCommon();
        setupDualArmMode();
        setupJoystickSubscriber();
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

    void UnifiedTargetTrajectoriesInteractiveMarker::setupJoystickSubscriber()
    {
        auto joystickCallback = [this](const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            this->joystickCallback(msg);
        };
        joystickSubscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, joystickCallback);

        if (mode_ == Mode::SINGLE_ARM)
        {
            RCLCPP_INFO(node_->get_logger(), "🎮 Joystick subscriber created for single arm mode");
            RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control is DISABLED by default. Press Y button to enable.");
            RCLCPP_INFO(node_->get_logger(),
                        "🎮 Controls: Y=toggle joystick, X=toggle continuous mode, A=send position (non-continuous)");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "🎮 Joystick subscriber created for dual arm mode");
            RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control is DISABLED by default. Press Y button to enable.");
            RCLCPP_INFO(node_->get_logger(),
                        "🎮 Controls: Y=toggle joystick, X=toggle continuous mode, A=send position (non-continuous), B=switch active arm")
            ;
        }
    }

    visualization_msgs::msg::InteractiveMarker
    UnifiedTargetTrajectoriesInteractiveMarker::createSingleArmMarker() const
    {
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = "world";
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = "Goal";
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = "Right click to send command or toggle continuous mode";

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
        interactiveMarker.header.frame_id = "world";
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = isLeftArm ? "LeftArmGoal" : "RightArmGoal";
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = (isLeftArm ? "Left" : "Right") + std::string(
            " arm target - Right click to send command");

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
            std::lock_guard<std::mutex> lock(markerPoseMutex_);
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
            std::lock_guard<std::mutex> lock(markerPoseMutex_);
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

        // 更新marker显示
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

    void UnifiedTargetTrajectoriesInteractiveMarker::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check button input
        if (msg->buttons.size() > 3)
        {
            auto currentTime = node_->now();
            double timeSinceLastButton = (currentTime - lastButtonTime_).seconds();
            bool cooldownActive = timeSinceLastButton < buttonCooldownDuration_;

            // Check if any functional button is pressed
            bool yPressed = msg->buttons[3];
            bool xPressed = msg->buttons[2];
            bool aPressed = msg->buttons[0] && !continuousMode_;
            bool bPressed = msg->buttons[1]; // B button to switch active arm (dual arm mode)

            // If any button is pressed and no button was pressed before
            if ((yPressed || xPressed || aPressed || bPressed) && !anyButtonPressed_)
            {
                if (!cooldownActive)
                {
                    lastButtonTime_ = currentTime;

                    // Y button to enable/disable joystick control (button 3)
                    if (yPressed)
                    {
                        joystickEnabled_ = !joystickEnabled_; // Toggle state

                        if (joystickEnabled_)
                        {
                            // When switching to joystick mode, use current marker position
                            {
                                std::lock_guard lock(markerPoseMutex_);
                                if (mode_ == Mode::SINGLE_ARM)
                                {
                                    joystickPosition_ = singleArmPosition_;
                                    joystickOrientation_ = singleArmOrientation_;
                                }
                                else
                                {
                                    // Dual arm mode: use current active arm position
                                    const auto& currentPos = (activeArm_ == ArmType::LEFT)
                                                                 ? leftArmPosition_
                                                                 : rightArmPosition_;
                                    const auto& currentOrient = (activeArm_ == ArmType::LEFT)
                                                                    ? leftArmOrientation_
                                                                    : rightArmOrientation_;
                                    joystickPosition_ = currentPos;
                                    joystickOrientation_ = currentOrient;
                                }
                            }
                            RCLCPP_INFO(node_->get_logger(),
                                        "🎮 Joystick control ENABLED! Starting from current marker position.");
                            if (mode_ == Mode::SINGLE_ARM)
                            {
                                RCLCPP_INFO(node_->get_logger(), "🎮 Current position: [%.3f, %.3f, %.3f]",
                                            singleArmPosition_.x(), singleArmPosition_.y(), singleArmPosition_.z());
                            }
                            else
                            {
                                RCLCPP_INFO(node_->get_logger(), "🎮 Active arm: %s",
                                            (activeArm_ == ArmType::LEFT) ? "LEFT" : "RIGHT");
                            }
                        }
                        else
                        {
                            RCLCPP_INFO(node_->get_logger(),
                                        "🎮 Joystick control DISABLED! Manual marker control restored.");
                        }
                    }

                    // X button to toggle continuous input mode (button 2)
                    if (xPressed)
                    {
                        togglePublishMode();
                    }

                    // A button to send current position in non-continuous mode (button 0)
                    if (aPressed)
                    {
                        if (mode_ == Mode::SINGLE_ARM)
                        {
                            sendSingleArmTrajectories();
                            RCLCPP_INFO(node_->get_logger(), "🎮 Sending single arm position via A button.");
                        }
                        else
                        {
                            sendDualArmTrajectories();
                            RCLCPP_INFO(node_->get_logger(), "🎮 Sending dual arm positions via A button.");
                        }
                    }

                    // B button to switch active arm (dual arm mode, button 1)
                    if (bPressed && mode_ == Mode::DUAL_ARM)
                    {
                        activeArm_ = activeArm_ == ArmType::LEFT ? ArmType::RIGHT : ArmType::LEFT;
                        RCLCPP_INFO(node_->get_logger(), "🎮 Switched active arm to: %s",
                                    activeArm_ == ArmType::LEFT ? "LEFT" : "RIGHT");

                        // Update joystick position to newly active arm position
                        if (joystickEnabled_)
                        {
                            std::lock_guard lock(markerPoseMutex_);
                            const auto& currentPos = activeArm_ == ArmType::LEFT
                                                         ? leftArmPosition_
                                                         : rightArmPosition_;
                            const auto& currentOrient = activeArm_ == ArmType::LEFT
                                                            ? leftArmOrientation_
                                                            : rightArmOrientation_;
                            joystickPosition_ = currentPos;
                            joystickOrientation_ = currentOrient;
                        }
                    }
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(), "🎮 Button cooldown active (%.2f seconds remaining)",
                                 buttonCooldownDuration_ - timeSinceLastButton);
                }
            }

            // Update button state
            anyButtonPressed_ = yPressed || xPressed || aPressed || bPressed;
        }

        // If joystick control is enabled, check update frequency
        if (joystickEnabled_)
        {
            auto currentTime = node_->now();
            double timeSinceLastUpdate = (currentTime - lastJoystickUpdateTime_).seconds();
            double updateInterval = 1.0 / joystickUpdateRate_;

            // If too short time since last update, skip this update
            if (timeSinceLastUpdate < updateInterval)
            {
                return;
            }

            lastJoystickUpdateTime_ = currentTime;
            // Reference moveit_teleop mapping
            // Right stick controls position (axes[3], axes[4], axes[5])
            // axes[4]: up/down movement (Z-axis) - RIGHT_STICK_Y
            // axes[3]: left/right movement (Y-axis) - RIGHT_STICK_X
            // axes[5]: forward/backward movement (X-axis) - using triggers
            if (msg->axes.size() > 5)
            {
                // Check if there is valid joystick input (avoid deadzone)
                bool hasValidInput = false;

                // Triggers control forward/backward movement (X-axis)
                double lin_x_right = -0.5 * (msg->axes[5] - 1.0); // RIGHT_TRIGGER
                double lin_x_left = 0.5 * (msg->axes[2] - 1.0); // LEFT_TRIGGER
                double x_movement = (lin_x_right + lin_x_left) * joystickLinearScale_;

                if (std::abs(msg->axes[4]) > 0.1 || std::abs(msg->axes[3]) > 0.1 ||
                    std::abs(lin_x_right) > 0.1 || std::abs(lin_x_left) > 0.1)
                {
                    hasValidInput = true;
                    // Only update position when there is valid input
                    joystickPosition_.x() += x_movement;
                    joystickPosition_.y() += msg->axes[3] * joystickLinearScale_; // Right stick X-axis
                    joystickPosition_.z() += msg->axes[4] * joystickLinearScale_; // Right stick Y-axis
                }

                // Left stick controls orientation (axes[0], axes[1])
                // axes[1]: rotation around Y-axis (pitch) - LEFT_STICK_Y
                // axes[0]: rotation around X-axis (roll) - LEFT_STICK_X
                // Buttons control rotation around Z-axis (yaw)
                double pitch = 0.0;
                double roll = 0.0;
                double yaw = 0.0;

                // Check left stick input
                if (msg->axes.size() > 1)
                {
                    if (std::abs(msg->axes[0]) > 0.1 || std::abs(msg->axes[1]) > 0.1)
                    {
                        hasValidInput = true;
                        pitch = msg->axes[1] * joystickAngularScale_; // Left stick Y-axis
                        roll = msg->axes[0] * joystickAngularScale_; // Left stick X-axis
                    }
                }

                // Buttons control yaw (independent of stick input)
                if (msg->buttons.size() > 5)
                {
                    if (msg->buttons[5])
                    {
                        // RIGHT_BUMPER
                        hasValidInput = true;
                        yaw = joystickAngularScale_;
                    }
                    if (msg->buttons[4])
                    {
                        // LEFT_BUMPER
                        hasValidInput = true;
                        yaw = -joystickAngularScale_;
                    }
                }

                // If there is any rotation input, update orientation
                if (std::abs(pitch) > 0.001 || std::abs(roll) > 0.001 || std::abs(yaw) > 0.001)
                {
                    // Create rotation increment
                    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());

                    Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;
                    joystickOrientation_ = joystickOrientation_ * rotationIncrement;
                    joystickOrientation_.normalize();
                }

                // Only update marker when there is valid input
                if (hasValidInput)
                {
                    // Update marker position
                    {
                        std::lock_guard lock(markerPoseMutex_);
                        if (mode_ == Mode::SINGLE_ARM)
                        {
                            singleArmPosition_ = joystickPosition_;
                            singleArmOrientation_ = joystickOrientation_;
                        }
                        else
                        {
                            // Dual arm mode: only update current active arm
                            if (activeArm_ == ArmType::LEFT)
                            {
                                leftArmPosition_ = joystickPosition_;
                                leftArmOrientation_ = joystickOrientation_;
                            }
                            else
                            {
                                rightArmPosition_ = joystickPosition_;
                                rightArmOrientation_ = joystickOrientation_;
                            }
                        }
                    }

                    // Update marker display in RViz
                    geometry_msgs::msg::Pose markerPose;
                    markerPose.position.x = joystickPosition_.x();
                    markerPose.position.y = joystickPosition_.y();
                    markerPose.position.z = joystickPosition_.z();
                    markerPose.orientation.w = joystickOrientation_.w();
                    markerPose.orientation.x = joystickOrientation_.x();
                    markerPose.orientation.y = joystickOrientation_.y();
                    markerPose.orientation.z = joystickOrientation_.z();

                    if (mode_ == Mode::SINGLE_ARM)
                    {
                        server_->setPose("Goal", markerPose);
                    }
                    else
                    {
                        // Dual arm mode: update current active arm marker
                        std::string markerName = activeArm_ == ArmType::LEFT ? "LeftArmGoal" : "RightArmGoal";
                        server_->setPose(markerName, markerPose);
                    }
                    server_->applyChanges();

                    // Output debug information
                    RCLCPP_DEBUG(node_->get_logger(), "🎮 Updated %s marker position: [%.3f, %.3f, %.3f]",
                                 mode_ == Mode::SINGLE_ARM ? "single arm" :
                                 activeArm_ == ArmType::LEFT ? "left arm" : "right arm",
                                 joystickPosition_.x(), joystickPosition_.y(), joystickPosition_.z());
                }
            }
        }
    }
} // namespace ocs2
