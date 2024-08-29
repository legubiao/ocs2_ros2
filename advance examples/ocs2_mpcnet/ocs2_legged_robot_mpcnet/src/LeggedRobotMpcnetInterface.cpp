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

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetInterface.h"


#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpcnet_core/control/MpcnetOnnxController.h>
#include <ocs2_raisim_core/RaisimRollout.h>
#include <ocs2_raisim_core/RaisimRolloutSettings.h>

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>

namespace ocs2::legged_robot {
    LeggedRobotMpcnetInterface::LeggedRobotMpcnetInterface(size_t nDataGenerationThreads,
                                                           size_t nPolicyEvaluationThreads, bool raisim) {
        // create ONNX environment
        auto onnxEnvironmentPtr = mpcnet::createOnnxEnvironment();
        // paths to files
        const std::string taskFile = ament_index_cpp::get_package_share_directory("ocs2_legged_robot") +
                                     "/config/mpc/task.info";
        const std::string urdfFile = ament_index_cpp::get_package_share_directory("ocs2_robotic_assets") +
                                     "/resources/anymal_c/urdf/anymal.urdf";
        const std::string referenceFile = ament_index_cpp::get_package_share_directory("ocs2_legged_robot") +
                                          "/config/command/reference.info";
        const std::string raisimFile = ament_index_cpp::get_package_share_directory("ocs2_legged_robot_raisim") +
                                       "/config/raisim.info";
        const std::string resourcePath = ament_index_cpp::get_package_share_directory("ocs2_robotic_assets") +
                                         "/resources/anymal_c/meshes";

        // set up MPC-Net rollout manager for data generation and policy evaluation
        std::vector<std::unique_ptr<MPC_BASE> > mpcPtrs;
        std::vector<std::unique_ptr<mpcnet::MpcnetControllerBase> > mpcnetPtrs;
        std::vector<std::unique_ptr<RolloutBase> > rolloutPtrs;
        std::vector<std::shared_ptr<mpcnet::MpcnetDefinitionBase> > mpcnetDefinitionPtrs;
        std::vector<std::shared_ptr<ReferenceManagerInterface> > referenceManagerPtrs;

        mpcPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
        mpcnetPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
        rolloutPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
        mpcnetDefinitionPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
        referenceManagerPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);

        for (int i = 0; i < (nDataGenerationThreads + nPolicyEvaluationThreads); i++) {
            interfaces_.push_back(
                std::make_unique<LeggedRobotInterface>(taskFile, urdfFile, referenceFile));

            auto definition = std::make_shared<LeggedRobotMpcnetDefinition>(*interfaces_[i]);
            mpcPtrs.push_back(getMpc(*interfaces_[i]));
            mpcnetPtrs.push_back(std::make_unique<mpcnet::MpcnetOnnxController>(
                definition, interfaces_[i]->getReferenceManagerPtr(),
                onnxEnvironmentPtr));

            if (raisim) {
                RaisimRolloutSettings rollout_settings(raisimFile, "rollout");
                rollout_settings.portNumber_ += i;

                conversions_ptrs.push_back(std::make_unique<LeggedRobotRaisimConversions>(
                    interfaces_[i]->getPinocchioInterface(),
                    interfaces_[i]->getCentroidalModelInfo(),
                    interfaces_[i]->getInitialState()));

                conversions_ptrs[i]->loadSettings(raisimFile, "rollout", true);
                rolloutPtrs.push_back(std::make_unique<RaisimRollout>(
                    urdfFile, resourcePath,
                    [&, i](const vector_t &state, const vector_t &input) {
                        return conversions_ptrs[i]->stateToRaisimGenCoordGenVel(state, input);
                    },
                    [&, i](const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
                        return conversions_ptrs[i]->raisimGenCoordGenVelToState(q, dq);
                    },
                    [&, i](double time, const vector_t &input, const vector_t &state, const Eigen::VectorXd &q,
                           const Eigen::VectorXd &dq) {
                        return conversions_ptrs[i]->inputToRaisimGeneralizedForce(
                            time, input, state, q, dq);
                    },
                    nullptr, rollout_settings,
                    [&, i](double time, const vector_t &input, const vector_t &state, const Eigen::VectorXd &q,
                           const Eigen::VectorXd &dq) {
                        return conversions_ptrs[i]->inputToRaisimPdTargets(
                            time, input, state, q, dq);
                    }));

                if (rollout_settings.generateTerrain_) {
                    raisim::TerrainProperties properties;
                    properties.zScale = rollout_settings.terrainRoughness_;
                    properties.seed = rollout_settings.terrainSeed_ + i;
                    properties.heightOffset = -0.4;
                    auto terrainPtr = dynamic_cast<RaisimRollout *>(rolloutPtrs[i].get())->generateTerrain(
                        properties);
                    conversions_ptrs[i]->setTerrain(*terrainPtr);
                }
            } else {
                rolloutPtrs.push_back(
                    std::unique_ptr<RolloutBase>(interfaces_[i]->getRollout().clone()));
            }
            mpcnetDefinitionPtrs.push_back(definition);
            referenceManagerPtrs.push_back(interfaces_[i]->getReferenceManagerPtr());
        }
        mpcnetRolloutManagerPtr_ = std::make_unique<mpcnet::MpcnetRolloutManager>(
            nDataGenerationThreads, nPolicyEvaluationThreads,
            std::move(mpcPtrs), std::move(mpcnetPtrs), std::move(rolloutPtrs),
            mpcnetDefinitionPtrs, referenceManagerPtrs);
    }


    std::unique_ptr<MPC_BASE> LeggedRobotMpcnetInterface::getMpc(const LeggedRobotInterface &leggedRobotInterface) {
        // ensure MPC and DDP settings are as needed for MPC-Net
        const auto mpcSettings = [&] {
            auto settings = leggedRobotInterface.mpcSettings();
            settings.debugPrint_ = false;
            settings.coldStart_ = false;
            return settings;
        }();
        const auto ddpSettings = [&] {
            auto settings = leggedRobotInterface.ddpSettings();
            settings.algorithm_ = ddp::Algorithm::SLQ;
            settings.nThreads_ = 1;
            settings.displayInfo_ = false;
            settings.displayShortSummary_ = false;
            settings.checkNumericalStability_ = false;
            settings.debugPrintRollout_ = false;
            settings.useFeedbackPolicy_ = true;
            return settings;
        }();
        // create one MPC instance
        auto mpcPtr =
                std::make_unique<GaussNewtonDDP_MPC>(mpcSettings, ddpSettings, leggedRobotInterface.getRollout(),
                                                     leggedRobotInterface.getOptimalControlProblem(),
                                                     leggedRobotInterface.getInitializer());
        mpcPtr->getSolverPtr()->setReferenceManager(leggedRobotInterface.getReferenceManagerPtr());
        return mpcPtr;
    }
}
