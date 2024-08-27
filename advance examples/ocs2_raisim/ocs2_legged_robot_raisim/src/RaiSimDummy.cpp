/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
// Created by tlab-uav on 8/20/24.
//

#include <ocs2_legged_robot_raisim/RaiSimDummy.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>

namespace ocs2::legged_robot {
    void RaiSimDummy::publishGridMap(const std::string &frameId) const {
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

    RaiSimDummy::RaiSimDummy(const std::string &robot_name) : Node(robot_name + "_raisim_dummy",
                                                                   rclcpp::NodeOptions()
                                                                   .allow_undeclared_parameters(true)
                                                                   .automatically_declare_parameters_from_overrides(
                                                                       true)) {
        gridmapPublisher_ = create_publisher<grid_map_msgs::msg::GridMap>("raisim_heightmap", 1);

        taskFile_ = get_parameter("taskFile").as_string();
        urdfFile_ = get_parameter("urdfFile").as_string();
        referenceFile_ = get_parameter("referenceFile").as_string();
        raisimFile_ = get_parameter("raisimFile").as_string();
        resourcePath_ = get_parameter("resourcePath").as_string();


        // Legged robot interface
        interface_ = std::make_shared<LeggedRobotInterface>(taskFile_, urdfFile_, referenceFile_);


        // RaiSim rollout
        conversions_ = std::make_shared<LeggedRobotRaisimConversions>(interface_->getPinocchioInterface(),
                                                                      interface_->getCentroidalModelInfo(),
                                                                      interface_->getInitialState());
        conversions_->loadSettings(raisimFile_, "rollout", true);
        RaisimRolloutSettings rollout_settings(raisimFile_, "rollout", true);
        rollout_ = std::make_shared<RaisimRollout>(
            urdfFile_, resourcePath_,
            [&](const vector_t &state, const vector_t &input) {
                return conversions_->stateToRaisimGenCoordGenVel(state, input);
            },
            [&](const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
                return conversions_->raisimGenCoordGenVelToState(q, dq);
            },
            [&](double time, const vector_t &input, const vector_t &state,
                const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
                return conversions_->inputToRaisimGeneralizedForce(
                    time, input, state, q,
                    dq);
            },
            nullptr, rollout_settings,
            [&](double time, const vector_t &input, const vector_t &state,
                const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
                return conversions_->inputToRaisimPdTargets(
                    time, input, state, q, dq);
            });

        // terrain
        if (rollout_settings.generateTerrain_) {
            raisim::TerrainProperties properties;
            properties.zScale = rollout_settings.terrainRoughness_;
            properties.seed = rollout_settings.terrainSeed_;
            properties.heightOffset = -0.4;
            terrainPtr_ = std::unique_ptr<raisim::HeightMap>(rollout_->generateTerrain(properties));
            publishGridMap("odom");
        }

        // Visualizations
        const CentroidalModelPinocchioMapping mapping(interface_->getCentroidalModelInfo());
        PinocchioEndEffectorKinematics kinematics(interface_->getPinocchioInterface(), mapping,
                                                  interface_->modelSettings().contactNames3DoF);
        visualizer_ = std::make_shared<RaiSimVisualizer>(interface_->getPinocchioInterface(),
                                                         interface_->getCentroidalModelInfo(),
                                                         kinematics, SharedPtr(this));
        // visualizer_->updateTerrain();
        RCLCPP_INFO(get_logger(), "RaiSimDummy initialized");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::string robot_name = "legged_robot";
    const auto node = std::make_shared<ocs2::legged_robot::RaiSimDummy>(robot_name);

    // mrt
    ocs2::MRT_ROS_Interface mrt(robot_name);
    mrt.initRollout(node->rollout_.get());
    mrt.launchNodes(node);

    // Legged robot RaiSim Dummy Loop
    ocs2::MRT_ROS_Dummy_Loop loop(mrt, node->interface_->mpcSettings().mrtDesiredFrequency_,
                                  node->interface_->mpcSettings().mpcDesiredFrequency_);
    loop.subscribeObservers({node->visualizer_});

    // Initial State
    ocs2::SystemObservation observation;
    observation.mode = ocs2::legged_robot::STANCE;
    observation.time = 0.0;
    observation.state = node->interface_->getInitialState();
    observation.input = ocs2::vector_t::Zero(node->interface_->getCentroidalModelInfo().inputDim);

    // Initial command
    ocs2::TargetTrajectories targetTrajectories(
        {observation.time}, {observation.state}, {observation.input});

    // Run the RaiSim Dummy Loop
    loop.run(observation, targetTrajectories);

    return 0;
}
