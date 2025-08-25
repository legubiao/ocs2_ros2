#include "ocs2_ros_interfaces/command/MarkerAutoPositionWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2
{
    MarkerAutoPositionWrapper::MarkerAutoPositionWrapper(
        rclcpp::Node::SharedPtr node,
        std::string topicPrefix,
        IMarkerControl* markerControl,
        const UpdateMode updateMode,
        const bool dualArmMode,
        const double cooldownDuration,
        const double maxUpdateFrequency
    )
        : node_(std::move(node)),
          markerControl_(markerControl),
          topicPrefix_(std::move(topicPrefix)),
          updateMode_(updateMode),
          cooldownDuration_(cooldownDuration),
          maxUpdateFrequency_(maxUpdateFrequency),
          minUpdateInterval_(1.0 / maxUpdateFrequency),
          dualArmMode_(dualArmMode),

          initialized_(false),
          updateEnabled_(true),
          lastMpcObservationTime_(node_->now()),
          lastUpdateTime_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
          // Set to very early time to avoid frequency limit on first check
          leftArmPoseReceived_(false),
          rightArmPoseReceived_(false)
    {
        // Create pose subscribers
        if (dualArmMode_)
        {
            // Dual arm mode: subscribe to left and right arm end effector poses
            leftEndEffectorPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                topicPrefix_ + "_left_end_effector_pose", 1,
                std::bind(&MarkerAutoPositionWrapper::leftEndEffectorPoseCallback, this, std::placeholders::_1));

            rightEndEffectorPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                topicPrefix_ + "_right_end_effector_pose", 1,
                std::bind(&MarkerAutoPositionWrapper::rightEndEffectorPoseCallback, this, std::placeholders::_1));
        }
        else
        {
            // Single arm mode: subscribe to left arm end effector pose (default to left arm)
            leftEndEffectorPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                topicPrefix_ + "_left_end_effector_pose", 1,
                std::bind(&MarkerAutoPositionWrapper::leftEndEffectorPoseCallback, this, std::placeholders::_1));
        }

        // Create MPC observation subscriber
        observationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
            topicPrefix_ + "_mpc_observation", 1,
            std::bind(&MarkerAutoPositionWrapper::observationCallback, this, std::placeholders::_1));

        // Create cooldown check timer
        cooldownCheckTimer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0), // Check every second
            std::bind(&MarkerAutoPositionWrapper::checkCooldownCallback, this));
    }


    bool MarkerAutoPositionWrapper::processPoseCallback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg, bool isLeftArm)
    {
        auto currentTime = node_->now();

        // Check if update should occur first, avoid calling shouldUpdatePosition inside lock
        bool shouldUpdate = false;
        {
            std::lock_guard lock(stateMutex_);

            // Check frequency limit
            double timeSinceLastUpdate = (currentTime - lastUpdateTime_).seconds();

            if (timeSinceLastUpdate >= minUpdateInterval_)
            {
                switch (updateMode_)
                {
                case UpdateMode::DISABLED:
                    break;

                case UpdateMode::INITIALIZATION:
                    shouldUpdate = !initialized_;
                    break;

                case UpdateMode::CONTINUOUS:
                    shouldUpdate = updateEnabled_ || !initialized_;
                    break;

                default:
                    break;
                }
            }

            // Update state
            if (isLeftArm)
            {
                leftArmPoseReceived_ = true;
            }
            else
            {
                rightArmPoseReceived_ = true;
            }
        }

        return shouldUpdate;
    }

    void MarkerAutoPositionWrapper::leftEndEffectorPoseCallback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        if (bool shouldUpdate = processPoseCallback(msg, true); !shouldUpdate)
        {
            return;
        }

        updateLeftArmMarkerPosition(msg);

        // Handle INITIALIZATION mode in single-arm mode
        if (!dualArmMode_)
        {
            handleInitializationMode();
        }
    }

    void MarkerAutoPositionWrapper::rightEndEffectorPoseCallback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        bool shouldUpdate = processPoseCallback(msg, false);
        if (!shouldUpdate)
        {
            return;
        }

        updateRightArmMarkerPosition(msg);
    }

    void MarkerAutoPositionWrapper::observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg)
    {
        auto currentTime = node_->now();
        lastMpcObservationTime_ = currentTime;

        // Disable marker position updates when MPC observation is received (enter cooldown)
        std::lock_guard lock(stateMutex_);
        updateEnabled_ = false;
    }

    void MarkerAutoPositionWrapper::checkCooldownCallback()
    {
        std::lock_guard lock(stateMutex_);

        // Check if updates should be re-enabled
        bool shouldEnable = false;

        if (updateMode_ == UpdateMode::INITIALIZATION)
        {
            // Initialization mode: enable only when not initialized
            shouldEnable = !initialized_;
        }
        else if (updateMode_ == UpdateMode::CONTINUOUS)
        {
            // Continuous mode: includes initialization and re-enabling after cooldown
            if (!initialized_)
            {
                // Initialization phase: enable directly
                shouldEnable = true;
            }
            else if (!updateEnabled_)
            {
                // After cooldown: re-enable if no MPC observation received for cooldown duration
                shouldEnable = (node_->now() - lastMpcObservationTime_).seconds() > cooldownDuration_;
            }
        }

        if (shouldEnable && !updateEnabled_)
        {
            updateEnabled_ = true;
            RCLCPP_INFO(node_->get_logger(), "Marker auto position re-enabled after cooldown period");
        }
    }


    void MarkerAutoPositionWrapper::updateMarkerPosition(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg,
        IMarkerControl::ArmType armType,
        const std::string& markerName)
    {
        if (!markerControl_)
        {
            RCLCPP_WARN(node_->get_logger(), "Marker control not available");
            return;
        }

        // Convert pose to Eigen types
        Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                       msg->pose.orientation.y, msg->pose.orientation.z);

        // Update marker based on mode
        if (dualArmMode_)
        {
            markerControl_->setDualArmPose(armType, position, orientation);
            markerControl_->updateMarkerDisplay(markerName, position, orientation);
        }
        else
        {
            markerControl_->setSingleArmPose(position, orientation);
            markerControl_->updateMarkerDisplay("Goal", position, orientation);
        }
    }

    void MarkerAutoPositionWrapper::handleInitialization(bool isLeftArm)
    {
        if (dualArmMode_)
        {
            // In dual-arm mode, only set initialized when both arms have received messages
            if (isLeftArm)
            {
                if (rightArmPoseReceived_ && !initialized_)
                {
                    initialized_ = true;
                }
            }
            else
            {
                if (leftArmPoseReceived_ && !initialized_)
                {
                    initialized_ = true;
                }
            }
        }
        else
        {
            // In single-arm mode, initialize immediately
            if (!initialized_)
            {
                initialized_ = true;
            }

            // In single-arm mode, update lastUpdateTime_
            lastUpdateTime_ = node_->now();
        }
    }

    void MarkerAutoPositionWrapper::handleInitializationMode()
    {
        // Decide whether to disable based on update mode
        std::lock_guard lock(stateMutex_);
        if (updateMode_ == UpdateMode::INITIALIZATION)
        {
            // Initialization mode: disable after update
            updateEnabled_ = false;
        }
        // Other modes: maintain current state, managed by cooldown mechanism
    }

    void MarkerAutoPositionWrapper::updateLeftArmMarkerPosition(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        updateMarkerPosition(msg, IMarkerControl::ArmType::LEFT, "LeftArmGoal");
        handleInitialization(true);

        // Handle INITIALIZATION mode in single-arm mode
        if (!dualArmMode_)
        {
            handleInitializationMode();
        }
    }

    void MarkerAutoPositionWrapper::updateRightArmMarkerPosition(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        updateMarkerPosition(msg, IMarkerControl::ArmType::RIGHT, "RightArmGoal");
        handleInitialization(false);

        // Update last update time
        lastUpdateTime_ = node_->now();

        // Handle INITIALIZATION mode
        handleInitializationMode();
    }
} // namespace ocs2
