#include "ocs2_ros_interfaces/command/VRMarkerWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ocs2
{

        VRMarkerWrapper::VRMarkerWrapper(
        rclcpp::Node::SharedPtr node,
        IMarkerControl* markerControl,
        const double updateRate)
        : node_(std::move(node)),
          markerControl_(markerControl),
          updateRate_(updateRate),
          enabled_(false),
          lastUpdateTime_(node_->now()),
          currentPosition_(0.0, 0.0, 1.0),
          currentOrientation_(1.0, 0.0, 0.0, 0.0)
    {
        // Create VR subscriber
        auto vrLeftCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->vrLeftCallback(msg);
        };
        subLeft_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "xr_left_ee_pose", 10, vrLeftCallback);

        auto vrRightCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->vrRightCallback(msg);
        };
        subRight_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "xr_right_ee_pose", 10, vrRightCallback);

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRMarkerWrapper created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default. Press right stick to enable.");

        // Initialize the marker control mode to continuous mode
        markerControl_->togglePublishMode();
    }

    bool VRMarkerWrapper::check_node_exists(const std::shared_ptr<rclcpp::Node>& node, const std::string& target_node_name)
    {
        std::vector<std::string> node_names = node->get_node_graph_interface()->get_node_names();

        for (const auto& name : node_names)
        {
            // std::cout << "Discovered node: " << name << std::endl;
            if (name == target_node_name)
            {
                return true;
            }
        }
        return false;
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
    
    void VRMarkerWrapper::vrLeftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

        if (check_node_exists(node_, XR_NODE_NAME) && !enabled_.load())
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "🕹️🕶️🕹️ xr_target_node found, VR control ENABLED!");
            this->enable();
        }
        else if (!check_node_exists(node_, XR_NODE_NAME) && enabled_.load())
        {
            this->disable();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "🕹️🕶️🕹️ xr_target_node not found, VR control DISABLED!");
            return;
        }

        leftEEPose_ = poseMsgToMatrix(msg);
        matrixToPosOri(leftEEPose_, leftPosition_, leftOrientation_);
        if (enabled_.load())
        {
            std::cout << "Left Position: " << leftPosition_.transpose() << std::endl;
            // Update left arm
            this->updateMarkerPose(leftPosition_, leftOrientation_, IMarkerControl::ArmType::LEFT);

        }
    }

    void VRMarkerWrapper::vrRightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        rightEEPose_ = poseMsgToMatrix(msg);
        matrixToPosOri(rightEEPose_, rightPosition_, rightOrientation_);
        if (enabled_.load())
        {
            if (markerControl_->getMode() == IMarkerControl::Mode::DUAL_ARM)
            {
                // Dual arm mode: always update right arm
                this->updateMarkerPose(rightPosition_, rightOrientation_, IMarkerControl::ArmType::RIGHT);
            }
        }
    }

    // Eigen::Matrix4d VRMarkerWrapper::getLeftPose() const
    // {
    //     return leftEEPose_;
    // }

    // Eigen::Matrix4d VRMarkerWrapper::getRightPose() const
    // {
    //     return rightEEPose_;
    // }


    void VRMarkerWrapper::updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const IMarkerControl::ArmType targetArm)
    {
        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
        {
            markerControl_->setSingleArmPose(position, orientation);
            markerControl_->updateMarkerDisplay("Goal", position, orientation);
        }
        else
        {
            // Dual arm mode: update current active arm
            // const auto activeArm = markerControl_->getActiveArm();
            markerControl_->setDualArmPose(targetArm, position, orientation);

            const std::string markerName =
                (targetArm == IMarkerControl::ArmType::LEFT) ? "LeftArmGoal" : "RightArmGoal";
            markerControl_->updateMarkerDisplay(markerName, position, orientation);
        }

        // Output debug information
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Updated %s marker position: [%.3f, %.3f, %.3f]",
                     markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM ? "single arm" :
                     targetArm == IMarkerControl::ArmType::LEFT ? "left arm" : "right arm",
                     position.x(), position.y(), position.z());
    }

    // void VRMarkerWrapper::syncExternalPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    // {
    //     // Always update the internal position and orientation, regardless of enabled state
    //     currentPosition_ = position;
    //     currentOrientation_ = orientation;
        
    //     // Log the sync operation
    //     RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Synced external position: [%.3f, %.3f, %.3f] (enabled: %s)",
    //                  position.x(), position.y(), position.z(),
    //                  enabled_.load() ? "true" : "false");
    // }

    // void VRMarkerWrapper::syncCurrentPoseWithMarker()
    // {
    //     if (!markerControl_)
    //     {
    //         RCLCPP_WARN(node_->get_logger(), "🕹️🕶️🕹️ Marker control not available for pose sync");
    //         return;
    //     }

    //     // Get current marker position based on mode
    //     if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
    //     {
    //         auto [pos, orient] = markerControl_->getSingleArmPose();
    //         currentPosition_ = pos;
    //         currentOrientation_ = orient;
    //     }
    //     else
    //     {
    //         auto [pos, orient] = markerControl_->getDualArmPose(markerControl_->getActiveArm());
    //         currentPosition_ = pos;
    //         currentOrientation_ = orient;
    //     }

    //     RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Synced current pose with marker: [%.3f, %.3f, %.3f]",
    //                  currentPosition_.x(), currentPosition_.y(), currentPosition_.z());
    // }

    Eigen::Matrix4d VRMarkerWrapper::poseMsgToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = msg->pose.position.x;
    pose(1, 3) = msg->pose.position.y;
    pose(2, 3) = msg->pose.position.z;

    Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
    pose.block<3, 3>(0, 0) = rot;

    return pose;
    }


    void VRMarkerWrapper::matrixToPosOri(const Eigen::Matrix4d& matrix, Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
    {
        position = matrix.block<3, 1>(0, 3);
        Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
        orientation = Eigen::Quaterniond(rot);
    }
}   // namespace ocs2