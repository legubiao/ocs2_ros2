#include "ocs2_ros_interfaces/command/JoystickMarkerWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ocs2
{
    JoystickMarkerWrapper::JoystickMarkerWrapper(
        rclcpp::Node::SharedPtr node,
        IMarkerControl* markerControl,
        const double linearScale,
        const double angularScale,
        const double updateRate)
        : node_(std::move(node)),
          markerControl_(markerControl),
          linearScale_(linearScale),
          angularScale_(angularScale),
          updateRate_(updateRate),
          enabled_(false),
          lastUpdateTime_(node_->now()),
          lastButtonTime_(node_->now()),
          buttonCooldownDuration_(0.5),
          anyButtonPressed_(false),
          currentPosition_(0.0, 0.0, 1.0),
          currentOrientation_(1.0, 0.0, 0.0, 0.0)
    {
        // Initialize button states
        for (bool& lastButtonState : lastButtonStates_)
        {
            lastButtonState = false;
        }

        // Create joystick subscriber
        auto joystickCallback = [this](const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            this->joystickCallback(msg);
        };
        joystickSubscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, joystickCallback);

        RCLCPP_INFO(node_->get_logger(), "🎮 JoystickMarkerWrapper created");
        RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control is DISABLED by default. Press Y button to enable.");
        RCLCPP_INFO(node_->get_logger(),
                    "🎮 Controls: Y=toggle joystick, X=toggle continuous mode, A=send position, B=switch active arm (dual arm mode)")
        ;
    }

    void JoystickMarkerWrapper::enable()
    {
        enabled_.store(true);
        RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control ENABLED!");
    }

    void JoystickMarkerWrapper::disable()
    {
        enabled_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control DISABLED!");
    }

    void JoystickMarkerWrapper::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check update frequency
        auto currentTime = node_->now();
        double timeSinceLastUpdate = (currentTime - lastUpdateTime_).seconds();
        double updateInterval = 1.0 / updateRate_;

        if (timeSinceLastUpdate < updateInterval)
        {
            return;
        }
        lastUpdateTime_ = currentTime;

        // Process buttons first
        processButtons(msg);

        // Process axes if enabled
        if (enabled_.load())
        {
            processAxes(msg);
        }
    }

    void JoystickMarkerWrapper::processButtons(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons.size() <= 5)
        {
            return;
        }

        auto currentTime = node_->now();
        double timeSinceLastButton = (currentTime - lastButtonTime_).seconds();
        bool cooldownActive = timeSinceLastButton < buttonCooldownDuration_;

        // Check button states
        bool yPressed = msg->buttons[3]; // Y button
        bool xPressed = msg->buttons[2]; // X button
        bool aPressed = msg->buttons[0]; // A button
        bool bPressed = msg->buttons[1]; // B button

        // Detect button press events (rising edge)
        bool yJustPressed = yPressed && !lastButtonStates_[3];
        bool xJustPressed = xPressed && !lastButtonStates_[2];
        bool aJustPressed = aPressed && !lastButtonStates_[0];
        bool bJustPressed = bPressed && !lastButtonStates_[1];

        // Update last button states
        lastButtonStates_[0] = aPressed;
        lastButtonStates_[1] = bPressed;
        lastButtonStates_[2] = xPressed;
        lastButtonStates_[3] = yPressed;

        // Process button events
        if ((yJustPressed || xJustPressed || aJustPressed || bJustPressed) && !anyButtonPressed_)
        {
            if (!cooldownActive)
            {
                lastButtonTime_ = currentTime;

                // Y button: toggle joystick control
                if (yJustPressed)
                {
                    if (enabled_.load())
                    {
                        disable();
                    }
                    else
                    {
                        enable();
                        // Initialize joystick position to current marker position
                        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
                        {
                            auto [pos, orient] = markerControl_->getSingleArmPose();
                            currentPosition_ = pos;
                            currentOrientation_ = orient;
                        }
                        else
                        {
                            auto [pos, orient] = markerControl_->getDualArmPose(markerControl_->getActiveArm());
                            currentPosition_ = pos;
                            currentOrientation_ = orient;
                        }
                    }
                }

                // X button: toggle continuous mode
                if (xJustPressed && enabled_.load())
                {
                    markerControl_->togglePublishMode();
                }

                // A button: send trajectory
                if (aJustPressed && enabled_.load() && !markerControl_->isContinuousMode())
                {
                    if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
                    {
                        markerControl_->sendSingleArmTrajectories();
                        RCLCPP_INFO(node_->get_logger(), "🎮 Sending single arm position via A button.");
                    }
                    else
                    {
                        markerControl_->sendDualArmTrajectories();
                        RCLCPP_INFO(node_->get_logger(), "🎮 Sending dual arm positions via A button.");
                    }
                }

                // B button: switch active arm (dual arm mode only)
                if (bJustPressed && enabled_.load() && markerControl_->getMode() == IMarkerControl::Mode::DUAL_ARM)
                {
                    auto currentArm = markerControl_->getActiveArm();
                    auto newArm = currentArm == IMarkerControl::ArmType::LEFT
                                      ? IMarkerControl::ArmType::RIGHT
                                      : IMarkerControl::ArmType::LEFT;
                    markerControl_->setActiveArm(newArm);

                    // Update joystick position to newly active arm position
                    auto [pos, orient] = markerControl_->getDualArmPose(newArm);
                    currentPosition_ = pos;
                    currentOrientation_ = orient;

                    RCLCPP_INFO(node_->get_logger(), "🎮 Switched active arm to: %s",
                                (newArm == IMarkerControl::ArmType::LEFT) ? "LEFT" : "RIGHT");
                }
            }
        }

        // Update button state tracking
        anyButtonPressed_ = yPressed || xPressed || aPressed || bPressed;
    }

    void JoystickMarkerWrapper::processAxes(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= 5)
        {
            return;
        }

        bool hasValidInput = false;

        // Right stick controls position (axes[3], axes[4], axes[5])
        // axes[4]: up/down movement (Z-axis) - RIGHT_STICK_Y
        // axes[3]: left/right movement (Y-axis) - RIGHT_STICK_X
        // axes[5]: forward/backward movement (X-axis) - using triggers

        // Triggers control forward/backward movement (X-axis)
        double lin_x_right = -0.5 * (msg->axes[5] - 1.0); // RIGHT_TRIGGER
        double lin_x_left = 0.5 * (msg->axes[2] - 1.0); // LEFT_TRIGGER
        double x_movement = (lin_x_right + lin_x_left) * linearScale_;

        if (std::abs(msg->axes[4]) > 0.1 || std::abs(msg->axes[3]) > 0.1 ||
            std::abs(lin_x_right) > 0.1 || std::abs(lin_x_left) > 0.1)
        {
            hasValidInput = true;
            // Update position
            currentPosition_.x() += x_movement;
            currentPosition_.y() += msg->axes[3] * linearScale_; // Right stick X-axis
            currentPosition_.z() += msg->axes[4] * linearScale_; // Right stick Y-axis
        }

        // Left stick controls orientation (axes[0], axes[1])
        // axes[1]: rotation around Y-axis (pitch) - LEFT_STICK_Y
        // axes[0]: rotation around X-axis (roll) - LEFT_STICK_X
        double pitch = 0.0;
        double roll = 0.0;
        double yaw = 0.0;

        if (msg->axes.size() > 1)
        {
            if (std::abs(msg->axes[0]) > 0.1 || std::abs(msg->axes[1]) > 0.1)
            {
                hasValidInput = true;
                pitch = msg->axes[1] * angularScale_; // Left stick Y-axis
                roll = msg->axes[0] * angularScale_; // Left stick X-axis
            }
        }

        // Buttons control yaw (independent of stick input)
        if (msg->buttons.size() > 5)
        {
            if (msg->buttons[5])
            {
                // RIGHT_BUMPER
                hasValidInput = true;
                yaw = angularScale_;
            }
            if (msg->buttons[4])
            {
                // LEFT_BUMPER
                hasValidInput = true;
                yaw = -angularScale_;
            }
        }

        // Update orientation if there is rotation input
        if (std::abs(pitch) > 0.001 || std::abs(roll) > 0.001 || std::abs(yaw) > 0.001)
        {
            // Create rotation increment
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());

            Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;
            currentOrientation_ = currentOrientation_ * rotationIncrement;
            currentOrientation_.normalize();
        }

        // Update marker if there is valid input
        if (hasValidInput)
        {
            updateMarkerPose(currentPosition_, currentOrientation_);
        }
    }

    void JoystickMarkerWrapper::updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
        {
            markerControl_->setSingleArmPose(position, orientation);
            markerControl_->updateMarkerDisplay("Goal", position, orientation);
        }
        else
        {
            // Dual arm mode: update current active arm
            const auto activeArm = markerControl_->getActiveArm();
            markerControl_->setDualArmPose(activeArm, position, orientation);

            const std::string markerName =
                (activeArm == IMarkerControl::ArmType::LEFT) ? "LeftArmGoal" : "RightArmGoal";
            markerControl_->updateMarkerDisplay(markerName, position, orientation);
        }

        // Output debug information
        RCLCPP_DEBUG(node_->get_logger(), "🎮 Updated %s marker position: [%.3f, %.3f, %.3f]",
                     markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM ? "single arm" :
                     markerControl_->getActiveArm() == IMarkerControl::ArmType::LEFT ? "left arm" : "right arm",
                     position.x(), position.y(), position.z());
    }
} // namespace ocs2
