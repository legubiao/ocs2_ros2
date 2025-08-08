/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_ros_interfaces/command/UnifiedTargetTrajectoriesInteractiveMarker.h>
#include <ocs2_ros_interfaces/command/JoystickMarkerWrapper.h>
#include <ocs2_ros_interfaces/command/MarkerAutoPositionWrapper.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

using namespace ocs2;

/**
 * 从taskFile中读取dualArmMode配置
 */
bool readDualArmModeFromTaskFile(const std::string& taskFile)
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        bool dualArmMode = false;
        // 尝试从endEffector或finalEndEffector配置中读取dualArmMode
        loadData::loadPtreeValue(pt, dualArmMode, "endEffector.dualArmMode", false);
        if (!dualArmMode)
        {
            loadData::loadPtreeValue(pt, dualArmMode, "finalEndEffector.dualArmMode", false);
        }

        return dualArmMode;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error reading dualArmMode from task file: " << e.what() << std::endl;
        return false;
    }
}

/**
 * 从taskFile中读取frame信息，根据manipulatorModelType决定使用哪个frame
 */
std::string getMarkerFrameFromTaskFile(const std::string& taskFile)
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        // 读取manipulatorModelType
        size_t manipulatorModelType = 1; // 默认为1 (WheelBasedMobileManipulator)
        loadData::loadPtreeValue(pt, manipulatorModelType, "model_information.manipulatorModelType", false);

        // 读取baseFrame
        std::string baseFrame = "base_link"; // 默认值
        loadData::loadPtreeValue(pt, baseFrame, "model_information.baseFrame", false);

        // 根据manipulatorModelType决定使用哪个frame
        if (manipulatorModelType == 0) // DefaultManipulator
        {
            // 当manipulatorModelType为0时，使用机器人的baseFrame
            return baseFrame;
        }
        else
        {
            // 其他情况使用world frame
            return "world";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error reading frame information from task file: " << e.what() << std::endl;
        return "world"; // 出错时使用默认的world frame
    }
}

/**
 * Converts the pose of the interactive marker to TargetTrajectories (single arm).
 */
TargetTrajectories goalPoseToTargetTrajectories(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
    const SystemObservation& observation)
{
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};
    // state trajectory: 3 + 4 for desired position vector and orientation
    // quaternion
    const vector_t target =
        (vector_t(7) << position, orientation.coeffs()).finished();
    const vector_array_t stateTrajectory{target};
    // input trajectory
    const vector_array_t inputTrajectory{
        vector_t::Zero(observation.input.size())
    };

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Converts the poses of dual arm interactive markers to TargetTrajectories.
 * This function combines both left and right arm target poses into a single trajectory.
 */
TargetTrajectories dualArmGoalPoseToTargetTrajectories(
    const Eigen::Vector3d& leftPosition, const Eigen::Quaterniond& leftOrientation,
    const Eigen::Vector3d& rightPosition, const Eigen::Quaterniond& rightOrientation,
    const SystemObservation& observation)
{
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};

    // state trajectory: 14 dimensions (7 for left arm + 7 for right arm)
    // [left_x, left_y, left_z, left_qw, left_qx, left_qy, left_qz,
    //  right_x, right_y, right_z, right_qw, right_qx, right_qy, right_qz]
    const vector_t target = (vector_t(14) <<
        leftPosition, leftOrientation.coeffs(),
        rightPosition, rightOrientation.coeffs()).finished();

    const vector_array_t stateTrajectory{target};

    // input trajectory
    const vector_array_t inputTrajectory{
        vector_t::Zero(observation.input.size())
    };

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[])
{
    const std::string robotName = "mobile_manipulator";
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_target",
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));

    // 从taskFile读取双臂模式配置
    std::string taskFile = node->get_parameter("taskFile").as_string();
    bool dualArmMode = readDualArmModeFromTaskFile(taskFile);

    // 读取enableDynamicFrame参数，如果外部没有传入则默认为false
    bool enableDynamicFrame = false;
    if (node->has_parameter("enableDynamicFrame"))
    {
        enableDynamicFrame = node->get_parameter("enableDynamicFrame").as_bool();
        RCLCPP_INFO(node->get_logger(), "enableDynamicFrame parameter found: %s", enableDynamicFrame ? "true" : "false");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "enableDynamicFrame parameter not found, using default: false");
    }

    // 读取frame信息
    std::string markerFrame = "world"; // 默认使用world frame
    if (enableDynamicFrame)
    {
        markerFrame = getMarkerFrameFromTaskFile(taskFile);
        RCLCPP_INFO(node->get_logger(), "Dynamic frame selection enabled. Using marker frame: %s", markerFrame.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Dynamic frame selection disabled. Using default frame: %s", markerFrame.c_str());
    }

    // 检查是否启用手柄控制
    bool enableJoystick = false;
    try
    {
        enableJoystick = node->get_parameter("enableJoystick").as_bool();
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException&)
    {
        // 参数未声明，使用默认值false
        enableJoystick = false;
    }

    // 检查是否启用自动位置更新
    bool enableAutoPosition = false;
    try
    {
        enableAutoPosition = node->get_parameter("enableAutoPosition").as_bool();
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException&)
    {
        // 参数未声明，使用默认值false
        enableAutoPosition = false;
    }

    std::unique_ptr<JoystickMarkerWrapper> joystickControl;
    std::unique_ptr<MarkerAutoPositionWrapper> autoPositionWrapper;

    if (dualArmMode)
    {
        // Create dual arm interactive marker
        RCLCPP_INFO(node->get_logger(), "Dual arm mode enabled - creating dual arm interactive markers");
        UnifiedTargetTrajectoriesInteractiveMarker targetPoseCommand(node, robotName,
                                                                     &dualArmGoalPoseToTargetTrajectories, 10.0, markerFrame);

        // 如果启用手柄控制，创建JoystickMarkerWrapper实例
        if (enableJoystick)
        {
            RCLCPP_INFO(node->get_logger(), "Joystick marker wrapper enabled");
            joystickControl = std::make_unique<JoystickMarkerWrapper>(node, &targetPoseCommand);
        }

        // 如果启用自动位置更新，创建MarkerAutoPositionWrapper实例
        if (enableAutoPosition)
        {
            RCLCPP_INFO(node->get_logger(), "Marker auto position wrapper enabled");
            autoPositionWrapper = std::make_unique<MarkerAutoPositionWrapper>(
                node, robotName, &targetPoseCommand,
                MarkerAutoPositionWrapper::UpdateMode::CONTINUOUS,
                dualArmMode); // dualArmMode
        }

        spin(node);
        return 0;
    }

    // Single arm mode
    RCLCPP_INFO(node->get_logger(), "Single arm mode enabled");
    UnifiedTargetTrajectoriesInteractiveMarker targetPoseCommand(node, robotName, &goalPoseToTargetTrajectories, 10.0, markerFrame);

    // 如果启用手柄控制，创建JoystickMarkerWrapper实例
    if (enableJoystick)
    {
        RCLCPP_INFO(node->get_logger(), "Joystick marker wrapper enabled");
        joystickControl = std::make_unique<JoystickMarkerWrapper>(node, &targetPoseCommand);
    }

    // 如果启用自动位置更新，创建MarkerAutoPositionWrapper实例
    if (enableAutoPosition)
    {
        RCLCPP_INFO(node->get_logger(), "Marker auto position wrapper enabled");
        autoPositionWrapper = std::make_unique<MarkerAutoPositionWrapper>(
            node, robotName, &targetPoseCommand,
            MarkerAutoPositionWrapper::UpdateMode::CONTINUOUS,  // updateMode
            dualArmMode,  // dualArmMode (false for single arm)
            3.0,          // cooldownDuration
            1.0           // maxUpdateFrequency
        );
    }

    spin(node);
    return 0;
}
