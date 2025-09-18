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
          currentOrientation_(1.0, 0.0, 0.0, 0.0),
          isUpdateMode_(false),
          lastThumbstickState_(false)
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

        // Create left thumbstick subscriber
        auto thumbstickCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->leftThumbstickCallback(msg);
        };
        subLeftThumbstick_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_left_thumbstick", 10, thumbstickCallback);

        // Create robot pose subscribers
        auto robotLeftCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->robotLeftPoseCallback(msg);
        };
        subRobotLeftPose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "unitree_g1_left_end_effector_pose", 10, robotLeftCallback);

        auto robotRightCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->robotRightPoseCallback(msg);
        };
        subRobotRightPose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "unitree_g1_right_end_effector_pose", 10, robotRightCallback);

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRMarkerWrapper created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default. Press right stick to enable.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Left thumbstick toggles between STORAGE and UPDATE modes.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ STORAGE mode: Store VR and robot base poses (no marker update)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ UPDATE mode: Calculate pose differences and update markers");

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
        // Initialize the marker control mode to continuous mode
        if (!markerControl_->isContinuousMode())
        {
            markerControl_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Marker control set to CONTINUOUS mode.");
        }
    }

    void VRMarkerWrapper::disable()
    {
        enabled_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control DISABLED!");
        if (markerControl_->isContinuousMode())
        {
            markerControl_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Marker control set to MANUAL mode.");
        }
    }

    void VRMarkerWrapper::leftThumbstickCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool currentThumbstickState = msg->data;
        bool lastState = lastThumbstickState_.load();
        
        // Detect rising edge (button press)
        if (currentThumbstickState && !lastState)
        {
            if (!isUpdateMode_.load())
            {
                // Switch to update mode - store current poses as base poses
                vrBaseLeftPosition_ = leftPosition_;
                vrBaseLeftOrientation_ = leftOrientation_;
                vrBaseRightPosition_ = rightPosition_;
                vrBaseRightOrientation_ = rightOrientation_;
                
                robotBaseLeftPosition_ = robotCurrentLeftPosition_;
                robotBaseLeftOrientation_ = robotCurrentLeftOrientation_;
                robotBaseRightPosition_ = robotCurrentRightPosition_;
                robotBaseRightOrientation_ = robotCurrentRightOrientation_;
                
                isUpdateMode_.store(true);
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Switched to UPDATE mode - Base poses stored!");
            }
            else
            {
                // Switch to storage mode
                isUpdateMode_.store(false);
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Switched to STORAGE mode - Ready to store new base poses!");
            }
        }
        
        lastThumbstickState_.store(currentThumbstickState);
    }

    void VRMarkerWrapper::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Matrix4d pose = poseMsgToMatrix(msg);
        matrixToPosOri(pose, robotCurrentLeftPosition_, robotCurrentLeftOrientation_);
        
        // Debug: Log robot pose changes (only in storage mode)
        static Eigen::Vector3d lastLoggedRobotPos = Eigen::Vector3d::Zero();
        if (!isUpdateMode_.load() && (robotCurrentLeftPosition_ - lastLoggedRobotPos).norm() > POSITION_THRESHOLD) // Log if moved more than threshold
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Robot Left pose changed: [%.3f, %.3f, %.3f]", 
                       robotCurrentLeftPosition_.x(), robotCurrentLeftPosition_.y(), robotCurrentLeftPosition_.z());
            lastLoggedRobotPos = robotCurrentLeftPosition_;
        }
    }

    void VRMarkerWrapper::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Matrix4d pose = poseMsgToMatrix(msg);
        matrixToPosOri(pose, robotCurrentRightPosition_, robotCurrentRightOrientation_);
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
        
        // Debug: Log VR pose changes (only in storage mode)
        static Eigen::Vector3d lastLoggedVRPos = Eigen::Vector3d::Zero();
        if (!isUpdateMode_.load() && (leftPosition_ - lastLoggedVRPos).norm() > POSITION_THRESHOLD) // Log if moved more than threshold
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR Left pose changed: [%.3f, %.3f, %.3f]", 
                       leftPosition_.x(), leftPosition_.y(), leftPosition_.z());
            lastLoggedVRPos = leftPosition_;
        }
        
        if (enabled_.load())
        {
            if (isUpdateMode_.load())
            {
                // Update mode: calculate pose based on difference and update marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;
                
                calculatePoseFromDifference(leftPosition_, leftOrientation_,
                                          vrBaseLeftPosition_, vrBaseLeftOrientation_,
                                          robotBaseLeftPosition_, robotBaseLeftOrientation_,
                                          calculatedPos, calculatedOri);
                
                // Check if calculated pose has changed significantly
                if (hasPoseChanged(calculatedPos, calculatedOri, prevCalculatedLeftPosition_, prevCalculatedLeftOrientation_))
                {
                    // Debug output
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left VR Base: [%.3f, %.3f, %.3f]", 
                                vrBaseLeftPosition_.x(), vrBaseLeftPosition_.y(), vrBaseLeftPosition_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left VR Current: [%.3f, %.3f, %.3f]", 
                                leftPosition_.x(), leftPosition_.y(), leftPosition_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Robot Base: [%.3f, %.3f, %.3f]", 
                                robotBaseLeftPosition_.x(), robotBaseLeftPosition_.y(), robotBaseLeftPosition_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Calculated: [%.3f, %.3f, %.3f]", 
                                calculatedPos.x(), calculatedPos.y(), calculatedPos.z());
                    
                    // Update left arm with calculated pose
                    updateMarkerPose(calculatedPos, calculatedOri, IMarkerControl::ArmType::LEFT);
                    
                    // Update previous calculated pose
                    prevCalculatedLeftPosition_ = calculatedPos;
                    prevCalculatedLeftOrientation_ = calculatedOri;
                }
            }
            else
            {
                // Storage mode: just store the VR pose, don't update marker
                // Update previous VR pose for change detection (no marker update)
                prevVRLeftPosition_ = leftPosition_;
                prevVRLeftOrientation_ = leftOrientation_;
            }
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
                if (isUpdateMode_.load())
                {
                    // Update mode: calculate pose based on difference and update marker
                    Eigen::Vector3d calculatedPos;
                    Eigen::Quaterniond calculatedOri;
                    
                    calculatePoseFromDifference(rightPosition_, rightOrientation_,
                                              vrBaseRightPosition_, vrBaseRightOrientation_,
                                              robotBaseRightPosition_, robotBaseRightOrientation_,
                                              calculatedPos, calculatedOri);
                    
                    // Check if calculated pose has changed significantly
                    if (hasPoseChanged(calculatedPos, calculatedOri, prevCalculatedRightPosition_, prevCalculatedRightOrientation_))
                    {
                        // Dual arm mode: update right arm with calculated pose
                        updateMarkerPose(calculatedPos, calculatedOri, IMarkerControl::ArmType::RIGHT);
                        
                        // Update previous calculated pose
                        prevCalculatedRightPosition_ = calculatedPos;
                        prevCalculatedRightOrientation_ = calculatedOri;
                    }
                }
                else
                {
                    // Storage mode: just store the VR pose, don't update marker
                    // Update previous VR pose for change detection (no marker update)
                    prevVRRightPosition_ = rightPosition_;
                    prevVRRightOrientation_ = rightOrientation_;
                }
            }
        }
    }


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
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Updated %s marker position: [%.3f, %.3f, %.3f]",
                     markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM ? "single arm" :
                     targetArm == IMarkerControl::ArmType::LEFT ? "left arm" : "right arm",
                     position.x(), position.y(), position.z());
    }

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

    bool VRMarkerWrapper::hasPoseChanged(const Eigen::Vector3d& currentPos, const Eigen::Quaterniond& currentOri,
                                        const Eigen::Vector3d& prevPos, const Eigen::Quaterniond& prevOri)
    {
        // Check position change
        double positionDiff = (currentPos - prevPos).norm();
        if (positionDiff > POSITION_THRESHOLD)
        {
            return true;
        }

        // Check orientation change using quaternion angle difference
        double orientationDiff = std::abs(currentOri.angularDistance(prevOri));
        if (orientationDiff > ORIENTATION_THRESHOLD)
        {
            return true;
        }

        return false;
    }

    void VRMarkerWrapper::calculatePoseFromDifference(const Eigen::Vector3d& vrCurrentPos, const Eigen::Quaterniond& vrCurrentOri,
                                                     const Eigen::Vector3d& vrBasePos, const Eigen::Quaterniond& vrBaseOri,
                                                     const Eigen::Vector3d& robotBasePos, const Eigen::Quaterniond& robotBaseOri,
                                                     Eigen::Vector3d& resultPos, Eigen::Quaterniond& resultOri)
    {
        // Calculate VR pose difference (transformation from base to current)
        Eigen::Vector3d vrPosDiff = vrCurrentPos - vrBasePos;
        Eigen::Quaterniond vrOriDiff = vrBaseOri.inverse() * vrCurrentOri;
        
        // Apply the same transformation to robot base pose
        resultPos = robotBasePos + vrPosDiff;
        resultOri = robotBaseOri * vrOriDiff;
        
        // Normalize quaternion to avoid drift
        resultOri.normalize();
    }
}   // namespace ocs2