/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

//
// Created by tlab-uav on 8/27/24.
//


#include <memory>
#include <ocs2_legged_robot_mpcnet/MpcnetDummy.h>
#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"
#include <ocs2_mpcnet_core/dummy/MpcnetDummyLoopRos.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>


namespace ocs2::legged_robot {
    void MpcNetDummy::publishGridMap(const std::string &frameId) const {
        grid_map_msgs::msg::GridMap gridMapMsg;

        gridMapMsg.header.frame_id = frameId;

        const auto xResolution =
                terrainPtr_->getXSize() / static_cast<double>(terrainPtr_->getXSamples());
        const auto yResolution =
                terrainPtr_->getYSize() / static_cast<double>(terrainPtr_->getYSamples());
        if (std::abs(xResolution - yResolution) > 1e-9) {
            throw std::runtime_error(
                "RaisimHeightmapRosConverter::convertHeightmapToGridmap - Resolution "
                "in x and y must be identical");
        }
        gridMapMsg.info.resolution = xResolution;

        gridMapMsg.info.length_x = terrainPtr_->getXSize();
        gridMapMsg.info.length_y = terrainPtr_->getYSize();

        gridMapMsg.info.pose.position.x = terrainPtr_->getCenterX();
        gridMapMsg.info.pose.position.y = terrainPtr_->getCenterY();
        gridMapMsg.info.pose.orientation.w = 1.0;

        gridMapMsg.layers.emplace_back("elevation");
        std_msgs::msg::Float32MultiArray dataArray;
        dataArray.layout.dim.resize(2);
        dataArray.layout.dim[0].label = "column_index";
        dataArray.layout.dim[0].stride = terrainPtr_->getHeightVector().size();
        dataArray.layout.dim[0].size = terrainPtr_->getXSamples();
        dataArray.layout.dim[1].label = "row_index";
        dataArray.layout.dim[1].stride = terrainPtr_->getYSamples();
        dataArray.layout.dim[1].size = terrainPtr_->getYSamples();
        dataArray.data.insert(dataArray.data.begin(),
                              terrainPtr_->getHeightVector().rbegin(),
                              terrainPtr_->getHeightVector().rend());
        gridMapMsg.data.push_back(dataArray);
        gridmapPublisher_->publish(gridMapMsg);
    }

    MpcNetDummy::MpcNetDummy(const std::string &robot_name) : Node(robot_name + "_mpcnet_dummy", rclcpp::NodeOptions()
                                                                   .allow_undeclared_parameters(true)
                                                                   .automatically_declare_parameters_from_overrides(
                                                                       true)) {
        RCLCPP_INFO(get_logger(), "Initializing MpcNetDummy");
        taskFile_ = get_parameter("taskFile").as_string();
        urdfFile_ = get_parameter("urdfFile").as_string();
        referenceFile_ = get_parameter("referenceFile").as_string();
        raisimFile_ = get_parameter("raisimFile").as_string();
        resourcePath_ = get_parameter("resourcePath").as_string();
        policyFile_ = get_parameter("policyFile").as_string();
        useRaisim = get_parameter("useRaisim").as_bool();
        robotName_ = robot_name;
    }

    void MpcNetDummy::init() {
        // Legged robot interface
        interface_ = std::make_shared<LeggedRobotInterface>(taskFile_, urdfFile_, referenceFile_);
        RCLCPP_INFO(get_logger(), "LeggedRobotInterface initialized");

        gaitReceiver_ = std::make_shared<GaitReceiver>(shared_from_this(),
                                                       interface_->getSwitchedModelReferenceManagerPtr()->
                                                       getGaitSchedule(), robotName_);
        RCLCPP_INFO(get_logger(), "GaitReceiver initialized");

        // ROS reference manager
        rosReferenceManager_ = std::make_shared<RosReferenceManager>(robotName_, interface_->getReferenceManagerPtr());
        rosReferenceManager_->subscribe(shared_from_this());
        RCLCPP_INFO(get_logger(), "RosReferenceManager initialized");

        // policy (MPC-Net controller)
        auto onnxEnvironmentPtr = mpcnet::createOnnxEnvironment();
        auto mpcnetDefinitionPtr = std::make_shared<LeggedRobotMpcnetDefinition>(*interface_);
        controller_ =
                std::make_unique<mpcnet::MpcnetOnnxController>(mpcnetDefinitionPtr, rosReferenceManager_,
                                                               onnxEnvironmentPtr);
        controller_->loadPolicyModel(policyFile_);

        // rollout
        if (useRaisim) {
            RCLCPP_INFO(get_logger(), "Using Raisim for rollout");
            gridmapPublisher_ = create_publisher<grid_map_msgs::msg::GridMap>("raisim_heightmap", 1);

            conversions_ = std::make_shared<LeggedRobotRaisimConversions>(interface_->getPinocchioInterface(),
                                                                          interface_->getCentroidalModelInfo(),
                                                                          interface_->getInitialState());
            conversions_->loadSettings(raisimFile_, "rollout", true);
            RaisimRolloutSettings rollout_settings(raisimFile_, "rollout", true);
            rollout_ = std::make_unique<RaisimRollout>(
                urdfFile_, resourcePath_,
                [&](const vector_t &state, const vector_t &input) {
                    return conversions_->stateToRaisimGenCoordGenVel(state, input);
                },
                [&](const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
                    return conversions_->raisimGenCoordGenVelToState(q, dq);
                },
                [&](double time, const vector_t &input, const vector_t &state, const Eigen::VectorXd &q,
                    const Eigen::VectorXd &dq) {
                    return conversions_->inputToRaisimGeneralizedForce(time, input, state, q, dq);
                },
                nullptr, rollout_settings,
                [&](double time, const vector_t &input, const vector_t &state, const Eigen::VectorXd &q,
                    const Eigen::VectorXd &dq) {
                    return conversions_->inputToRaisimPdTargets(time, input, state, q, dq);
                });

            if (rollout_settings.generateTerrain_) {
                raisim::TerrainProperties properties;
                properties.zScale = rollout_settings.terrainRoughness_;
                properties.seed = rollout_settings.terrainSeed_;
                properties.heightOffset = -0.4;
                auto raisimRolloutPtr = dynamic_cast<RaisimRollout *>(rollout_.get());
                terrainPtr_ = std::unique_ptr<raisim::HeightMap>(raisimRolloutPtr->generateTerrain(properties));
                publishGridMap("odom");
            }
        } else {
            RCLCPP_INFO(get_logger(), "Using dummy rollout");
            rollout_.reset(interface_->getRollout().clone());
        }

        // observer
        dummyObserverRos_ = std::make_shared<mpcnet::MpcnetDummyObserverRos>(shared_from_this(), robotName_);

        // visualization
        const CentroidalModelPinocchioMapping mapping(interface_->getCentroidalModelInfo());
        PinocchioEndEffectorKinematics kinematics(interface_->getPinocchioInterface(), mapping,
                                                  interface_->modelSettings().contactNames3DoF);
        visualizer_ = std::make_shared<RaiSimVisualizer>(interface_->getPinocchioInterface(),
                                                         interface_->getCentroidalModelInfo(),
                                                         kinematics, shared_from_this());

        RCLCPP_INFO(get_logger(), "MpcNetDummy initialized");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::string robot_name = "legged_robot";
    const auto node = std::make_shared<ocs2::legged_robot::MpcNetDummy>(robot_name);
    node->init();

    // MPC-Net dummy loop ROS
    const ocs2::scalar_t controlFrequency = node->interface_->mpcSettings().mrtDesiredFrequency_;
    const ocs2::scalar_t rosFrequency = node->interface_->mpcSettings().mpcDesiredFrequency_;
    ocs2::mpcnet::MpcnetDummyLoopRos dummy_loop_ros(controlFrequency, rosFrequency, std::move(node->controller_),
                                                    std::move(node->rollout_), node,
                                                    node->rosReferenceManager_);
    dummy_loop_ros.addObserver(node->dummyObserverRos_);
    dummy_loop_ros.addObserver(node->visualizer_);
    dummy_loop_ros.addSynchronizedModule(node->gaitReceiver_);

    // initial system observation
    ocs2::SystemObservation systemObservation;
    systemObservation.mode = ocs2::legged_robot::ModeNumber::STANCE;
    systemObservation.time = 0.0;
    systemObservation.state = node->interface_->getInitialState();
    systemObservation.input = ocs2::vector_t::Zero(node->interface_->getCentroidalModelInfo().inputDim);

    // initial target trajectories
    ocs2::TargetTrajectories targetTrajectories({systemObservation.time}, {systemObservation.state},
                                                {systemObservation.input});

    // run MPC-Net dummy loop ROS
    dummy_loop_ros.run(systemObservation, targetTrajectories);

    return 0;
}
