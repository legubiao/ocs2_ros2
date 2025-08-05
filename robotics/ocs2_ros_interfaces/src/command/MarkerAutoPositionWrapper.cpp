#include "ocs2_ros_interfaces/command/MarkerAutoPositionWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2
{
    MarkerAutoPositionWrapper::MarkerAutoPositionWrapper(
        rclcpp::Node::SharedPtr node,
        const std::string& topicPrefix,
        IMarkerControl* markerControl,
        const UpdateMode updateMode,
        const double cooldownDuration,
        const double maxUpdateFrequency)
        : node_(std::move(node)),
          markerControl_(markerControl),
          topicPrefix_(topicPrefix),
          updateMode_(updateMode),
          cooldownDuration_(cooldownDuration),
          maxUpdateFrequency_(maxUpdateFrequency),
          minUpdateInterval_(1.0 / maxUpdateFrequency),

          initialized_(false),
          updateEnabled_(true),
          lastMpcObservationTime_(node_->now()),
          lastEndEffectorPoseTime_(node_->now()),
          lastUpdateTime_(node_->now())
    {
        // Create end effector pose subscriber
        auto endEffectorPoseCallback = [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
        {
            this->endEffectorPoseCallback(msg);
        };
        endEffectorPoseSubscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            topicPrefix_ + "_end_effector_pose", 1, endEffectorPoseCallback);

        // Create MPC observation subscriber
        auto observationCallback = [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg)
        {
            this->observationCallback(msg);
        };
        observationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
            topicPrefix_ + "_mpc_observation", 1, observationCallback);

        // Create cooldown check timer
        cooldownCheckTimer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0), // Check every second
            std::bind(&MarkerAutoPositionWrapper::checkCooldownCallback, this));
    }


    void MarkerAutoPositionWrapper::setUpdateMode(UpdateMode mode)
    {
        updateMode_ = mode;
    }

    void MarkerAutoPositionWrapper::setCooldownDuration(double duration)
    {
        cooldownDuration_ = duration;
    }

    void MarkerAutoPositionWrapper::setMaxUpdateFrequency(double frequency)
    {
        maxUpdateFrequency_ = frequency;
        minUpdateInterval_ = 1.0 / frequency;
    }

    void MarkerAutoPositionWrapper::resetCooldown()
    {
        std::lock_guard lock(stateMutex_);
        updateEnabled_ = false;
        lastMpcObservationTime_ = node_->now();
    }

    void MarkerAutoPositionWrapper::endEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        auto currentTime = node_->now();
        lastEndEffectorPoseTime_ = msg->header.stamp;

        if (shouldUpdatePosition())
        {
            updateMarkerPosition(msg);
        }
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

    void MarkerAutoPositionWrapper::updateMarkerPosition(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
        {
            // Single arm mode
            Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                           msg->pose.orientation.y, msg->pose.orientation.z);

            markerControl_->setSingleArmPose(position, orientation);
            markerControl_->updateMarkerDisplay("Goal", position, orientation);

            if (!initialized_)
            {
                initialized_ = true;
            }
        }
        else
        {
            // Dual arm mode: use first position to initialize right arm
            Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                           msg->pose.orientation.y, msg->pose.orientation.z);

            markerControl_->setDualArmPose(IMarkerControl::ArmType::RIGHT, position, orientation);
            markerControl_->updateMarkerDisplay("RightArmGoal", position, orientation);

            if (!initialized_)
            {
                initialized_ = true;
            }
        }

        // Update last update time
        lastUpdateTime_ = node_->now();

        // Decide whether to disable based on update mode
        std::lock_guard lock(stateMutex_);
        if (updateMode_ == UpdateMode::INITIALIZATION)
        {
            // Initialization mode: disable after update
            updateEnabled_ = false;
        }
        // Other modes: maintain current state, managed by cooldown mechanism
    }

    bool MarkerAutoPositionWrapper::shouldUpdatePosition() const
    {
        std::lock_guard lock(stateMutex_);

        // Check frequency limit
        auto currentTime = node_->now();
        if ((currentTime - lastUpdateTime_).seconds() < minUpdateInterval_)
        {
            return false;
        }

        switch (updateMode_)
        {
        case UpdateMode::DISABLED:
            return false;

        case UpdateMode::INITIALIZATION:
            // Update only during initialization
            return !initialized_;

        case UpdateMode::CONTINUOUS:
            // Continuous updates (including initialization and after cooldown)
            return updateEnabled_ || !initialized_;

        default:
            return false;
        }
    }
} // namespace ocs2
