#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "rclcpp/rclcpp.hpp"

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv)
{
    const std::string robotName = "mobile_manipulator";

    // Initialize ros node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_mpc",
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));

    // Get node parameters
    std::string taskFile = node->get_parameter("taskFile").as_string();
    std::string libFolder = node->get_parameter("libFolder").as_string();
    std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;

    // Robot interface
    MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
        robotName, interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(node);

    // SQP MPC
    SqpMpc mpc(
        interface.mpcSettings(), interface.sqpSettings(),
        interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

    // Launch MPC ROS node
    MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes(node);

    // Successful exit
    return 0;
}
