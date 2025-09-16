#include "ocs2_ros_interfaces/command/VRMarkerWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ocs2
{

        VRMarkerWrapper::VRMarkerWrapper(
        rclcpp::Node::SharedPtr node,
        IMarkerControl* markerControl,
        const double linearScale,
        const double angularScale,
        const double updateRate,
        const JoystickMapping& mapping)
        : node_(std::move(node)),
          markerControl_(markerControl),
          linearScale_(linearScale),
          angularScale_(angularScale),
          updateRate_(updateRate),
          enabled_(false),
          lastUpdateTime_(node_->now()),
          currentPosition_(0.0, 0.0, 1.0),
          currentOrientation_(1.0, 0.0, 0.0, 0.0)
    {
        // TODO: need to defin new msg for VR controller
        // Create joystick subscriber
        auto vrCallback = [this](const char msg)
        {
            this->vrCallback(msg);
        };
        vrSubscriber_ = node_->create_subscription<char>(
            "vr", 10, vrCallback);

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRMarkerWrapper created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default. Press right stick to enable.");
    }

    void VRMarkerWrapper::enable()
    {
        enabled_.store(true);
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control ENABLED!");
    }

    void VRMarkerWrapper::disable()
    {
        enabled_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control DISABLED!");
    }

    void VRMarkerWrapper::vrCallback(const char msg)
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



    void VRMarkerWrapper::updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
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
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Updated %s marker position: [%.3f, %.3f, %.3f]",
                     markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM ? "single arm" :
                     markerControl_->getActiveArm() == IMarkerControl::ArmType::LEFT ? "left arm" : "right arm",
                     position.x(), position.y(), position.z());
    }


    void VRMarkerWrapper::syncExternalPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        // Always update the internal position and orientation, regardless of enabled state
        currentPosition_ = position;
        currentOrientation_ = orientation;
        
        // Log the sync operation
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Synced external position: [%.3f, %.3f, %.3f] (enabled: %s)",
                     position.x(), position.y(), position.z(),
                     enabled_.load() ? "true" : "false");
    }

    void VRMarkerWrapper::syncCurrentPoseWithMarker()
    {
        if (!markerControl_)
        {
            RCLCPP_WARN(node_->get_logger(), "🕹️🕶️🕹️ Marker control not available for pose sync");
            return;
        }

        // Get current marker position based on mode
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

        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Synced current pose with marker: [%.3f, %.3f, %.3f]",
                     currentPosition_.x(), currentPosition_.y(), currentPosition_.z());
    }

}