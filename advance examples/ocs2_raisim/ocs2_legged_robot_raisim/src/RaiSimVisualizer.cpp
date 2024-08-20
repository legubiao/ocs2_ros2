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

#include "ocs2_legged_robot_raisim/RaiSimVisualizer.h"
#include <grid_map_msgs/msg/grid_map.hpp>
#include <utility>

namespace ocs2::legged_robot {
    RaiSimVisualizer::RaiSimVisualizer(
        PinocchioInterface interface,
        CentroidalModelInfo model_info,
        const PinocchioEndEffectorKinematics &endEffectorKinematics,
        rclcpp::Node::SharedPtr node, scalar_t maxUpdateFrequency)
        : LeggedRobotVisualizer(std::move(interface), std::move(model_info),
                                endEffectorKinematics, node,
                                maxUpdateFrequency) {
    }

    void RaiSimVisualizer::update(const SystemObservation &observation,
                                  const PrimalSolution &primalSolution,
                                  const CommandData &command) {
        SystemObservation raisim_observation = observation;
        PrimalSolution primal_solution = primalSolution;
        CommandData raisim_command = command;
        // height relative to terrain
        if (terrainPtr_ != nullptr) {
            raisim_observation.state(8) +=
                    terrainPtr_->getHeight(observation.state(6), observation.state(7));
            for (size_t i = 0; i < primalSolution.stateTrajectory_.size(); i++) {
                primal_solution.stateTrajectory_[i](8) +=
                        terrainPtr_->getHeight(primalSolution.stateTrajectory_[i](6),
                                               primalSolution.stateTrajectory_[i](7));
            }
            raisim_command.mpcInitObservation_.state(8) +=
                    terrainPtr_->getHeight(command.mpcInitObservation_.state(6),
                                           command.mpcInitObservation_.state(7));
            for (size_t i = 0;
                 i < command.mpcTargetTrajectories_.stateTrajectory.size(); i++) {
                raisim_command.mpcTargetTrajectories_.stateTrajectory[i](8) +=
                        terrainPtr_->getHeight(
                            command.mpcTargetTrajectories_.stateTrajectory[i](6),
                            command.mpcTargetTrajectories_.stateTrajectory[i](7));
            }
        }
        LeggedRobotVisualizer::update(raisim_observation, primal_solution,
                                      raisim_command);
    }

    void RaiSimVisualizer::updateTerrain(const std::chrono::seconds timeout) {
        std::promise<std::shared_ptr<grid_map_msgs::msg::GridMap> > promise;
        auto future = promise.get_future();

        auto callback = [&promise](grid_map_msgs::msg::GridMap::SharedPtr msg) {
            promise.set_value(msg);
        };

        auto subscription = node_->create_subscription<grid_map_msgs::msg::GridMap>(
            "raisim_heightmap", 10, callback);

        if (future.wait_for(timeout) == std::future_status::timeout) {
            return;
        }

        const auto gridMap = future.get();

        if (gridMap->data[0].layout.dim[0].label != "column_index" or
            gridMap->data[0].layout.dim[1].label != "row_index") {
            throw std::runtime_error(
                "RaisimHeightmapRosConverter::convertGridmapToHeightmap - Layout of "
                "gridMap currently not supported");
        }

        const int xSamples = gridMap->data[0].layout.dim[0].size;
        const int ySamples = gridMap->data[0].layout.dim[1].size;
        const double xSize = gridMap->info.length_x;
        const double ySize = gridMap->info.length_y;
        const double centerX = gridMap->info.pose.position.x;
        const double centerY = gridMap->info.pose.position.y;

        std::vector<double> height(gridMap->data[0].data.rbegin(),
                                   gridMap->data[0].data.rend());

        terrainPtr_ = std::make_unique<raisim::HeightMap>(xSamples, ySamples, xSize, ySize,
                                                          centerX, centerY, height);
    }
} // namespace ocs2::legged_robot
