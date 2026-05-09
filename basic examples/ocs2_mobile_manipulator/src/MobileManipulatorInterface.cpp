/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <memory>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
#include "ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h"
#include "ocs2_mobile_manipulator/LogQuadraticPenalty.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/augmented_lagrangian/StateAugmentedLagrangian.h>
#include <ocs2_core/penalties/augmented/SlacknessSquaredHingePenalty.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorSe3LogConstraint.h"
#include "ocs2_mobile_manipulator/constraint/BodyRelativeConstraint.h"

#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/cost/QuadraticInputCost.h"
#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/FloatingArmManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/WheelBasedMobileManipulatorDynamics.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


namespace ocs2::mobile_manipulator
{
    MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile,
                                                           const std::string& libraryFolder,
                                                           const std::string& urdfFile)
    {
        // check that task file exists
        boost::filesystem::path taskFilePath(taskFile);
        if (boost::filesystem::exists(taskFilePath))
        {
            std::cerr << "[MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
        }
        else
        {
            throw std::invalid_argument(
                "[MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
        }
        // check that urdf file exists
        boost::filesystem::path urdfFilePath(urdfFile);
        if (boost::filesystem::exists(urdfFilePath))
        {
            std::cerr << "[MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
        }
        else
        {
            throw std::invalid_argument(
                "[MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
        }
        // create library folder if it does not exist
        boost::filesystem::path libraryFolderPath(libraryFolder);
        boost::filesystem::create_directories(libraryFolderPath);
        std::cerr << "[MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

        // read the task file
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        // resolve meta-information about the model
        // read manipulator type
        ManipulatorModelType modelType = loadManipulatorType(
            taskFile, "model_information.manipulatorModelType");
        // read the joints to make fixed
        std::vector<std::string> removeJointNames;
        loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
        // read the frame names
        std::string baseFrame, eeFrame, eeFrame1;
        loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
        loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
        loadData::loadPtreeValue<std::string>(pt, eeFrame1, "model_information.eeFrame1", false);

        std::cerr << "\n #### Model Information:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### model_information.manipulatorModelType: " << static_cast<int>(modelType);
        std::cerr << "\n #### model_information.removeJoints: ";
        for (const auto& name : removeJointNames)
        {
            std::cerr << "\"" << name << "\" ";
        }
        std::cerr << "\n #### Note: All mimic joints will be automatically detected and removed";
        std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
        std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"" << std::endl;
        std::cerr << " #### =============================================================================" <<
            std::endl;

        // create pinocchio interface
        pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
            createPinocchioInterface(urdfFile, modelType, removeJointNames));
        std::cerr << *pinocchioInterfacePtr_;

        // ManipulatorModelInfo
        manipulatorModelInfo_ = createManipulatorModelInfo(
            *pinocchioInterfacePtr_, modelType, baseFrame, eeFrame, eeFrame1);

        bool usePreComputation = true;
        bool recompileLibraries = true;
        std::cerr << "\n #### Model Settings:";
        std::cerr << "\n #### =============================================================================\n";
        loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
        loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
        std::cerr << " #### =============================================================================\n";

        // Default initial state
        initialState_.setZero(manipulatorModelInfo_.stateDim);
        const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
        const int armStateDim = manipulatorModelInfo_.armDim;

        // arm base DOFs initial state
        if (baseStateDim > 0)
        {
            vector_t initialBaseState = vector_t::Zero(baseStateDim);
            loadData::loadEigenMatrix(taskFile, "initialState.base." + modelTypeEnumToString(modelType),
                                      initialBaseState);
            initialState_.head(baseStateDim) = initialBaseState;
        }

        // arm joints DOFs velocity limits
        vector_t initialArmState = vector_t::Zero(armStateDim);
        loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
        initialState_.tail(armStateDim) = initialArmState;

        std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

        // DDP-MPC settings
        ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
        mpcSettings_ = mpc::loadSettings(taskFile, "mpc");
        const auto ompMaxThreads =
#ifdef _OPENMP
            static_cast<size_t>(omp_get_max_threads());
#else
            static_cast<size_t>(1);
#endif
        std::cerr << "DDP threads requested: " << ddpSettings_.nThreads_
                  << ", OpenMP max threads: " << ompMaxThreads << std::endl;

        // Reference Manager
        referenceManagerPtr_ = std::make_shared<ReferenceManager>();

        /*
         * Optimal control problem
         */
        // Cost
        problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

        // Constraints
        // joint limits constraint
        problem_.softConstraintPtr->add("jointLimits",
                                        getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
        // end-effector state constraint
        problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(
                                                 *pinocchioInterfacePtr_, taskFile, "endEffector",
                                                 usePreComputation, libraryFolder, recompileLibraries));
        problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(
                                                 *pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                 usePreComputation, libraryFolder, recompileLibraries));
        // self-collision avoidance constraint
        selfCollisionEnabled_ = true;
        loadData::loadPtreeValue(pt, selfCollisionEnabled_, "selfCollision.activate", true);
        if (selfCollisionEnabled_)
        {
            problem_.stateSoftConstraintPtr->add(
                "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile,
                                                            "selfCollision", usePreComputation,
                                                            libraryFolder, recompileLibraries));
        }

        // body relative constraint
        bool activateBodyRelative = false;
        loadData::loadPtreeValue(pt, activateBodyRelative, "bodyRelative.activate", false);
        if (activateBodyRelative)
        {
            problem_.stateSoftConstraintPtr->add(
                "bodyRelative", getBodyRelativeConstraint(*pinocchioInterfacePtr_, taskFile,
                                                          "bodyRelative", usePreComputation,
                                                          libraryFolder, recompileLibraries));
        }

        // joint 6/7 coupling constraint (only for 7-DOF arms)
        bool activateJoint67Coupling = false;
        loadData::loadPtreeValue(pt, activateJoint67Coupling, "joint67Coupling.activate", true);
        std::cerr << " #### [Joint67] taskFile=" << taskFile
                  << " activate=" << activateJoint67Coupling
                  << " armDim=" << manipulatorModelInfo_.armDim
                  << " stateDim=" << manipulatorModelInfo_.stateDim
                  << " inputDim=" << manipulatorModelInfo_.inputDim << std::endl;
        if (activateJoint67Coupling && (manipulatorModelInfo_.armDim == 7 || manipulatorModelInfo_.armDim == 14))
        {
            std::string constraintMode = "augmented_lagrangian";
            loadData::loadPtreeValue(pt, constraintMode, "joint67Coupling.mode", false);

            if (constraintMode == "soft") {
                std::cerr << " #### [Joint67] mode=SOFT (barrier penalty)\n";
                problem_.stateSoftConstraintPtr->add("joint67Coupling", getJoint67CouplingConstraint(taskFile));
                problem_.finalSoftConstraintPtr->add("joint67Coupling_terminal", getJoint67CouplingConstraint(taskFile));
            } else {
                std::cerr << " #### [Joint67] mode=AUGMENTED_LAGRANGIAN (near-hard inequality constraint)\n";
                auto alPtr = getJoint67CouplingAugmentedLagrangian(taskFile);
                // Support optional safety margin via shifting: h' = h - margin >= 0
                scalar_t margin = 0.0;
                loadData::loadPtreeValue(pt, margin, "joint67Coupling.margin", false);
                scalar_t smoothAbsEps = 1e-6;
                loadData::loadPtreeValue(pt, smoothAbsEps, "joint67Coupling.smoothAbsEps", false);

                class ShiftedStateConstraint final : public StateConstraint {
                public:
                    ShiftedStateConstraint(std::unique_ptr<StateConstraint> inner, scalar_t shift)
                        : StateConstraint(ConstraintOrder::Linear), inner_(std::move(inner)), shift_(shift) {}
                    ShiftedStateConstraint* clone() const override {
                        return new ShiftedStateConstraint(std::unique_ptr<StateConstraint>(inner_->clone()), shift_);
                    }
                    size_t getNumConstraints(scalar_t time) const override { return inner_->getNumConstraints(time); }
                    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& pre) const override {
                        return inner_->getValue(time, state, pre).array() - shift_;
                    }
                    VectorFunctionLinearApproximation getLinearApproximation(
                        scalar_t time, const vector_t& state, const PreComputation& pre) const override {
                        auto approx = inner_->getLinearApproximation(time, state, pre);
                        approx.f.array() -= shift_;
                        return approx;
                    }
                private:
                    std::unique_ptr<StateConstraint> inner_;
                    scalar_t shift_{0.0};
                };

                auto makeCouplingConstraint = [&]() -> std::unique_ptr<StateConstraint> {
                    std::unique_ptr<StateConstraint> c = std::make_unique<Joint67CouplingConstraint>(
                        manipulatorModelInfo_.stateDim, manipulatorModelInfo_.armDim, smoothAbsEps);
                    if (margin > 0.0) {
                        c = std::make_unique<ShiftedStateConstraint>(std::move(c), margin);
                    }
                    return c;
                };

                problem_.stateInequalityConstraintPtr->add("joint67Coupling", makeCouplingConstraint());
                problem_.stateInequalityLagrangianPtr->add("joint67Coupling", std::move(alPtr));

                problem_.finalInequalityConstraintPtr->add("joint67Coupling_terminal", makeCouplingConstraint());
                problem_.finalInequalityLagrangianPtr->add("joint67Coupling_terminal",
                    getJoint67CouplingAugmentedLagrangian(taskFile));
            }
        }
        else
        {
            std::cerr << " #### [Joint67] constraint SKIPPED\n";
        }

        // Rollout dynamics joint-limit baking (smooth outward velocity attenuation)
        bool bakeJointPositionLimits = false;
        scalar_t jointLimitEps = 0.02; // [rad] smoothing distance around limits
        loadData::loadPtreeValue(pt, bakeJointPositionLimits, "rollout.bakeJointPositionLimits", true);
        loadData::loadPtreeValue(pt, jointLimitEps, "rollout.jointLimitEps", true);

        vector_t armPosLower, armPosUpper;
        if (bakeJointPositionLimits)
        {
            const auto& model = pinocchioInterfacePtr_->getModel();
            armPosLower = model.lowerPositionLimit.tail(armStateDim);
            armPosUpper = model.upperPositionLimit.tail(armStateDim);
            std::cerr << " #### [Rollout] bakeJointPositionLimits=true, jointLimitEps=" << jointLimitEps << "\n";
        }

        // Dynamics
        switch (manipulatorModelInfo_.manipulatorModelType)
        {
        case ManipulatorModelType::DefaultManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<DefaultManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", armPosLower, armPosUpper, jointLimitEps, libraryFolder,
                    recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::FloatingArmManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<FloatingArmManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", armPosLower, armPosUpper, jointLimitEps, libraryFolder,
                    recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<FullyActuatedFloatingArmManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", armPosLower, armPosUpper, jointLimitEps,
                    libraryFolder, recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::WheelBasedMobileManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<WheelBasedMobileManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", armPosLower, armPosUpper, jointLimitEps, libraryFolder,
                    recompileLibraries, true);
                break;
            }
        default:
            throw std::invalid_argument("Invalid manipulator model type provided.");
        }

        /*
         * Pre-computation
         */
        if (usePreComputation)
        {
            problem_.preComputationPtr = std::make_unique<MobileManipulatorPreComputation>(
                *pinocchioInterfacePtr_, manipulatorModelInfo_);
        }

        // Rollout
        const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
        rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problem_.dynamicsPtr, rolloutSettings);

        // Initialization
        initializerPtr_ = std::make_unique<DefaultInitializer>(manipulatorModelInfo_.inputDim);
    }


    std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost(const std::string& taskFile)
    {
        matrix_t R = matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
        const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
        const int armStateDim = manipulatorModelInfo_.armDim;

        // arm base DOFs input costs
        if (baseInputDim > 0)
        {
            matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
            loadData::loadEigenMatrix(
                taskFile, "inputCost.R.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                R_base);
            R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
        }

        // arm joints DOFs input costs
        matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
        loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
        R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

        std::cerr << "\n #### Input Cost Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "inputCost.R:  \n" << R << '\n';
        std::cerr << " #### =============================================================================\n";

        return std::make_unique<QuadraticInputCost>(std::move(R), manipulatorModelInfo_.stateDim);
    }


    std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile, const std::string& prefix,
        bool usePreComputation, const std::string& libraryFolder,
        bool recompileLibraries)
    {
        scalar_t muPosition = 1.0;
        scalar_t muOrientation = 1.0;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### " << prefix << " Settings: ";
        std::cerr << "\n #### =============================================================================\n";

        // Read dual-arm mode configuration, default to false (single-arm mode)
        loadData::loadPtreeValue(pt, dual_arm_, prefix + ".dualArmMode", false);

        loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
        loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
        std::string trackingError = "pos_quat";  // legacy: position difference + SO(3) orientation error
        loadData::loadPtreeValue(pt, trackingError, prefix + ".trackingError", false);
        std::string se3Invariant = "right";  // for trackingError=se3_log
        loadData::loadPtreeValue(pt, se3Invariant, prefix + ".se3Invariant", false);
        std::string positionPenalty = "quadratic";
        scalar_t logPositionScale = 1.0;
        loadData::loadPtreeValue(pt, positionPenalty, prefix + ".positionPenalty", false);
        loadData::loadPtreeValue(pt, logPositionScale, prefix + ".logPositionScale", false);

        std::cerr << " #### Dual arm mode: " << (dual_arm_ ? "enabled" : "disabled") << std::endl;
        std::cerr << " #### =============================================================================\n";

        if (referenceManagerPtr_ == nullptr)
        {
            throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
        }

        std::unique_ptr<StateConstraint> constraint;
        bool logTrackingError = false;
        scalar_t logTrackingPeriod = 1.0;
        loadData::loadPtreeValue(pt, logTrackingError, prefix + ".logTrackingError", false);
        loadData::loadPtreeValue(pt, logTrackingPeriod, prefix + ".logTrackingPeriod", false);

        if (trackingError == "se3_log" && !usePreComputation)
        {
            std::cerr << "[WARN] " << prefix << ".trackingError=se3_log requires usePreComputation=true; "
                         "falling back to pos_quat.\n";
            trackingError = "pos_quat";
        }

        if (usePreComputation)
        {
            MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);

            if (dual_arm_)
            {
                // Dual-arm mode: create kinematics with two end effectors
                PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,
                                                            {
                                                                manipulatorModelInfo_.eeFrame,
                                                                manipulatorModelInfo_.eeFrame1
                                                            });
                if (trackingError == "se3_log")
                {
                    const auto inv = (se3Invariant == "left")
                                         ? EndEffectorSe3LogConstraint::InvariantType::Left
                                         : EndEffectorSe3LogConstraint::InvariantType::Right;
                    constraint = std::make_unique<EndEffectorSe3LogConstraint>(
                        std::vector<std::string>{manipulatorModelInfo_.eeFrame, manipulatorModelInfo_.eeFrame1},
                        *referenceManagerPtr_, true, inv);
                }
                else
                {
                    constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, true,
                                                                         logTrackingError, logTrackingPeriod);
                }
            }
            else
            {
                PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,
                                                            {manipulatorModelInfo_.eeFrame});
                if (trackingError == "se3_log")
                {
                    const auto inv = (se3Invariant == "left")
                                         ? EndEffectorSe3LogConstraint::InvariantType::Left
                                         : EndEffectorSe3LogConstraint::InvariantType::Right;
                    constraint = std::make_unique<EndEffectorSe3LogConstraint>(
                        std::vector<std::string>{manipulatorModelInfo_.eeFrame},
                        *referenceManagerPtr_, false, inv);
                }
                else
                {
                    constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, false,
                                                                         logTrackingError, logTrackingPeriod);
                }
            }
        }
        else
        {
            MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);

            if (dual_arm_)
            {
                // Dual-arm mode: create CppAd kinematics with two end effectors
                PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd,
                                                                 {
                                                                     manipulatorModelInfo_.eeFrame,
                                                                     manipulatorModelInfo_.eeFrame1
                                                                 },
                                                                 manipulatorModelInfo_.stateDim,
                                                                 manipulatorModelInfo_.inputDim,
                                                                 "end_effector_kinematics", libraryFolder,
                                                                 recompileLibraries, false);
                constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, true,
                                                                     logTrackingError, logTrackingPeriod);
            }
            else
            {
                PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd,
                                                                 {manipulatorModelInfo_.eeFrame},
                                                                 manipulatorModelInfo_.stateDim,
                                                                 manipulatorModelInfo_.inputDim,
                                                                 "end_effector_kinematics", libraryFolder,
                                                                 recompileLibraries, false);
                constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, false,
                                                                     logTrackingError, logTrackingPeriod);
            }
        }

        std::vector<std::unique_ptr<PenaltyBase>> penaltyArray;

        if (dual_arm_)
        {
            // Dual-arm mode: read dual-arm specific configuration, use default if not provided
            scalar_t leftMuPosition = muPosition;
            scalar_t leftMuOrientation = muOrientation;
            scalar_t rightMuPosition = muPosition;
            scalar_t rightMuOrientation = muOrientation;
            std::string leftPositionPenalty = positionPenalty;
            std::string rightPositionPenalty = positionPenalty;
            scalar_t leftLogPositionScale = logPositionScale;
            scalar_t rightLogPositionScale = logPositionScale;

            loadData::loadPtreeValue(pt, leftMuPosition, prefix + ".leftArm.muPosition", false);
            loadData::loadPtreeValue(pt, leftMuOrientation, prefix + ".leftArm.muOrientation", false);
            loadData::loadPtreeValue(pt, rightMuPosition, prefix + ".rightArm.muPosition", false);
            loadData::loadPtreeValue(pt, rightMuOrientation, prefix + ".rightArm.muOrientation", false);
            loadData::loadPtreeValue(pt, leftPositionPenalty, prefix + ".leftArm.positionPenalty", false);
            loadData::loadPtreeValue(pt, rightPositionPenalty, prefix + ".rightArm.positionPenalty", false);
            loadData::loadPtreeValue(pt, leftLogPositionScale, prefix + ".leftArm.logPositionScale", false);
            loadData::loadPtreeValue(pt, rightLogPositionScale, prefix + ".rightArm.logPositionScale", false);

            penaltyArray.resize(12);
            // Left arm: position + orientation
            std::generate_n(penaltyArray.begin(), 3, [&]() -> std::unique_ptr<PenaltyBase>
            {
                if (trackingError != "se3_log" && leftPositionPenalty == "log")
                {
                    return std::make_unique<LogQuadraticPenalty>(leftMuPosition, leftLogPositionScale);
                }
                return std::make_unique<QuadraticPenalty>(leftMuPosition);
            });
            std::generate_n(penaltyArray.begin() + 3, 3, [&]
            {
                return std::make_unique<QuadraticPenalty>(leftMuOrientation);
            });
            // Right arm: position + orientation
            std::generate_n(penaltyArray.begin() + 6, 3, [&]() -> std::unique_ptr<PenaltyBase>
            {
                if (trackingError != "se3_log" && rightPositionPenalty == "log")
                {
                    return std::make_unique<LogQuadraticPenalty>(rightMuPosition, rightLogPositionScale);
                }
                return std::make_unique<QuadraticPenalty>(rightMuPosition);
            });
            std::generate_n(penaltyArray.begin() + 9, 3, [&]
            {
                return std::make_unique<QuadraticPenalty>(rightMuOrientation);
            });
        }
        else
        {
            penaltyArray.resize(6);
            std::generate_n(penaltyArray.begin(), 3, [&]() -> std::unique_ptr<PenaltyBase>
            {
                if (trackingError != "se3_log" && positionPenalty == "log")
                {
                    return std::make_unique<LogQuadraticPenalty>(muPosition, logPositionScale);
                }
                return std::make_unique<QuadraticPenalty>(muPosition);
            });
            std::generate_n(penaltyArray.begin() + 3, 3, [&]
            {
                return std::make_unique<QuadraticPenalty>(muOrientation);
            });
        }

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
    }


    std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile, const std::string& urdfFile,
        const std::string& prefix, bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries)
    {
        std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
        std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t minimumDistance = 0.0;
        scalar_t activationDistance = -1.0;  // -1 means use default (5 * minimumDistance)

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### SelfCollision Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
        loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
        loadData::loadPtreeValue(pt, activationDistance, prefix + ".activationDistance", false);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
        
        // If activationDistance not specified, default to 5 * minimumDistance
        if (activationDistance < 0.0) {
            activationDistance = 5.0 * minimumDistance;
        }
        // Store distances for visualization and collision detection
        selfCollisionMinimumDistance_ = minimumDistance;
        selfCollisionActivationDistance_ = activationDistance;
        std::cerr << " #### minimumDistance: " << minimumDistance << " (minimum allowed distance)\n";
        std::cerr << " #### activationDistance: " << activationDistance << " (penalty only active when distance < this value)\n";
        std::cerr << " #### =============================================================================\n";

        PinocchioGeometryInterface geometryInterface(pinocchioInterface, urdfFile, collisionLinkPairs,
                                                     collisionObjectPairs);

        pinocchioGeometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(
            pinocchioInterface, urdfFile, collisionLinkPairs, collisionObjectPairs);

        const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
        std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

        std::unique_ptr<StateConstraint> constraint;
        if (usePreComputation)
        {
            constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(
                MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                std::move(geometryInterface), minimumDistance);
        }
        else
        {
            constraint = std::make_unique<SelfCollisionConstraintCppAd>(
                pinocchioInterface, MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                std::move(geometryInterface), minimumDistance,
                "self_collision", libraryFolder, recompileLibraries, false);
        }

        // Use ThresholdRelaxedBarrierPenalty with activation distance
        // The activationThreshold in penalty space is (activationDistance - minimumDistance)
        // because constraint value h = actual_distance - minimumDistance
        const scalar_t activationThreshold = activationDistance - minimumDistance;
        auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
            ThresholdRelaxedBarrierPenalty::Config{mu, delta, activationThreshold});

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }


    std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        bool activateJointPositionLimit = true;
        loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

        const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
        const int armStateDim = manipulatorModelInfo_.armDim;
        const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
        const int armInputDim = manipulatorModelInfo_.armDim;
        const auto& model = pinocchioInterface.getModel();

        // Load position limits
        std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
        if (activateJointPositionLimit)
        {
            scalar_t muPositionLimits = 1e-2;
            scalar_t deltaPositionLimits = 1e-3;
            scalar_t margin = 0.0; // [rad] shrink each limit inward for solver (reduces border clipping / monitor spikes)

            // arm joint DOF limits from the parsed URDF
            const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
            const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

            std::cerr << "\n #### JointPositionLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
            std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
            loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
            loadData::loadPtreeValue(pt, margin, "jointPositionLimits.margin", true);
            std::cerr << " #### margin (inward, rad): " << margin << std::endl;
            std::cerr << " #### =============================================================================\n";

            stateLimits.reserve(armStateDim);
            for (int i = 0; i < armStateDim; ++i)
            {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index = baseStateDim + i;
                scalar_t lo = lowerBound(i) + margin;
                scalar_t hi = upperBound(i) - margin;
                if (!(hi > lo))
                {
                    lo = lowerBound(i);
                    hi = upperBound(i);
                }
                boxConstraint.lowerBound = lo;
                boxConstraint.upperBound = hi;
                boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
                stateLimits.push_back(std::move(boxConstraint));
            }
        }

        // load velocity limits
        std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
        {
            vector_t lowerBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
            vector_t upperBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
            scalar_t muVelocityLimits = 1e-2;
            scalar_t deltaVelocityLimits = 1e-3;

            // Base DOFs velocity limits
            if (baseInputDim > 0)
            {
                vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
                vector_t upperBoundBase = vector_t::Zero(baseInputDim);
                loadData::loadEigenMatrix(taskFile,
                                          "jointVelocityLimits.lowerBound.base." + modelTypeEnumToString(
                                              manipulatorModelInfo_.manipulatorModelType),
                                          lowerBoundBase);
                loadData::loadEigenMatrix(taskFile,
                                          "jointVelocityLimits.upperBound.base." + modelTypeEnumToString(
                                              manipulatorModelInfo_.manipulatorModelType),
                                          upperBoundBase);
                lowerBound.head(baseInputDim) = lowerBoundBase;
                upperBound.head(baseInputDim) = upperBoundBase;
            }

            // arm joint DOFs velocity limits
            vector_t lowerBoundArm = vector_t::Zero(armInputDim);
            vector_t upperBoundArm = vector_t::Zero(armInputDim);
            loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
            loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
            lowerBound.tail(armInputDim) = lowerBoundArm;
            upperBound.tail(armInputDim) = upperBoundArm;

            std::cerr << "\n #### JointVelocityLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
            std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
            loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
            std::cerr << " #### =============================================================================\n";

            inputLimits.reserve(manipulatorModelInfo_.inputDim);
            for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i)
            {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index = i;
                boxConstraint.lowerBound = lowerBound(i);
                boxConstraint.upperBound = upperBound(i);
                boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
                inputLimits.push_back(std::move(boxConstraint));
            }
        }

        auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
        boxConstraints->initializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim),
                                         vector_t::Zero(manipulatorModelInfo_.stateDim));
        return boxConstraints;
    }

    std::unique_ptr<StateCost> MobileManipulatorInterface::getBodyRelativeConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile, const std::string& prefix,
        bool usePreComputation, const std::string& libraryFolder,
        bool recompileLibraries)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### " << prefix << " Settings: ";
        std::cerr << "\n #### =============================================================================\n";

        // Read configuration parameters
        std::string bodyLinkName = "base_link"; // default link name
        scalar_t rollTolerance = 0.1; // default roll tolerance (radians)
        scalar_t pitchTolerance = 0.1; // default pitch tolerance (radians)
        scalar_t muRoll = 1.0; // default roll penalty weight
        scalar_t muPitch = 1.0; // default pitch penalty weight

        // Position constraint weights for stability (XY plane only, Z free)
        scalar_t muPositionX = 0.5; // default X position penalty weight
        scalar_t muPositionY = 0.5; // default Y position penalty weight

        loadData::loadPtreeValue(pt, bodyLinkName, prefix + ".bodyLinkName", false);
        loadData::loadPtreeValue(pt, rollTolerance, prefix + ".rollTolerance", false);
        loadData::loadPtreeValue(pt, pitchTolerance, prefix + ".pitchTolerance", false);
        loadData::loadPtreeValue(pt, muRoll, prefix + ".muRoll", false);
        loadData::loadPtreeValue(pt, muPitch, prefix + ".muPitch", false);

        // Load position constraint weights
        loadData::loadPtreeValue(pt, muPositionX, prefix + ".muPositionX", false);
        loadData::loadPtreeValue(pt, muPositionY, prefix + ".muPositionY", false);

        std::cerr << " #### Body link name: " << bodyLinkName << std::endl;
        std::cerr << " #### Roll tolerance: " << rollTolerance << " rad (" << (rollTolerance * 180.0 / M_PI) << " deg)"
            << std::endl;
        std::cerr << " #### Pitch tolerance: " << pitchTolerance << " rad (" << (pitchTolerance * 180.0 / M_PI) <<
            " deg)" << std::endl;
        std::cerr << " #### Roll penalty weight: " << muRoll << std::endl;
        std::cerr << " #### Pitch penalty weight: " << muPitch << std::endl;
        std::cerr << " #### X position penalty weight: " << muPositionX << std::endl;
        std::cerr << " #### Y position penalty weight: " << muPositionY << std::endl;
        std::cerr << " #### =============================================================================\n";

        // Create the body relative constraint using our specialized BodyRelativeConstraint
        std::unique_ptr<StateConstraint> constraint;
        if (usePreComputation)
        {
            MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);

            // Single kinematics interface containing both end effector and base frame
            // Use the baseFrame already loaded in constructor
            PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,
                                                        {bodyLinkName, manipulatorModelInfo_.baseFrame});

            // Create our specialized constraint
            constraint = std::make_unique<BodyRelativeConstraint>(eeKinematics, bodyLinkName,
                                                                  rollTolerance, pitchTolerance,
                                                                  static_cast<int>(manipulatorModelInfo_.
                                                                      manipulatorModelType));
        }
        else
        {
            MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);

            // Create CppAd kinematics treating the body link as an end effector
            PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd,
                                                             {bodyLinkName, manipulatorModelInfo_.baseFrame},
                                                             manipulatorModelInfo_.stateDim,
                                                             manipulatorModelInfo_.inputDim,
                                                             "body_orientation_kinematics", libraryFolder,
                                                             recompileLibraries, false);

            // Create our specialized constraint
            constraint = std::make_unique<BodyRelativeConstraint>(eeKinematics, bodyLinkName,
                                                                  rollTolerance, pitchTolerance,
                                                                  static_cast<int>(manipulatorModelInfo_.
                                                                      manipulatorModelType));
        }

        // Create penalty array for BodyRelativeConstraint (4 constraints)
        // Roll, pitch, X position, Y position
        std::vector<std::unique_ptr<PenaltyBase>> penaltyArray;
        penaltyArray.resize(4);

        // Rotation constraints: roll and pitch
        penaltyArray[0] = std::make_unique<QuadraticPenalty>(muRoll); // Roll constraint (vertical orientation)
        penaltyArray[1] = std::make_unique<QuadraticPenalty>(muPitch); // Pitch constraint (vertical orientation)
        // Position constraints: XY for stability
        penaltyArray[2] = std::make_unique<QuadraticPenalty>(muPositionX); // X position (constrained for stability)
        penaltyArray[3] = std::make_unique<QuadraticPenalty>(muPositionY); // Y position (constrained for stability)

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
    }

    std::unique_ptr<StateCost> MobileManipulatorInterface::getJoint67CouplingConstraint(
        const std::string& taskFile)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### Joint67Coupling Settings: ";
        std::cerr << "\n #### =============================================================================\n";

        // Threshold relaxed barrier (inequality) on margin h = limit(|q7|) - |q6| >= 0
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t activationThreshold = 0.05;
        scalar_t margin = 0.0;
        scalar_t smoothAbsEps = 1e-6;
        // Support a few key variants for convenience/backward compatibility.
        loadData::loadPtreeValue(pt, mu, "joint67Coupling.mu", false);
        loadData::loadPtreeValue(pt, mu, "joint67Coupling.barrierMu", false);
        loadData::loadPtreeValue(pt, mu, "joint67Coupling.barrier.mu", false);

        loadData::loadPtreeValue(pt, delta, "joint67Coupling.delta", false);
        loadData::loadPtreeValue(pt, delta, "joint67Coupling.barrierDelta", false);
        loadData::loadPtreeValue(pt, delta, "joint67Coupling.barrier.delta", false);

        loadData::loadPtreeValue(pt, activationThreshold, "joint67Coupling.activationThreshold", false);
        loadData::loadPtreeValue(pt, activationThreshold, "joint67Coupling.barrierActivationThreshold", false);
        loadData::loadPtreeValue(pt, activationThreshold, "joint67Coupling.barrier.activationThreshold", false);

        loadData::loadPtreeValue(pt, margin, "joint67Coupling.margin", false);
        loadData::loadPtreeValue(pt, smoothAbsEps, "joint67Coupling.smoothAbsEps", false);

        std::cerr << " #### threshold barrier mu: " << mu << std::endl;
        std::cerr << " #### threshold barrier delta: " << delta << std::endl;
        std::cerr << " #### threshold barrier activationThreshold: " << activationThreshold << std::endl;
        std::cerr << " #### margin (shift): " << margin << std::endl;
        std::cerr << " #### smoothAbsEps: " << smoothAbsEps << std::endl;
        std::cerr << " #### =============================================================================\n";

        class ShiftedStateConstraint final : public StateConstraint {
        public:
            ShiftedStateConstraint(std::unique_ptr<StateConstraint> inner, scalar_t shift)
                : StateConstraint(ConstraintOrder::Linear), inner_(std::move(inner)), shift_(shift) {}
            ShiftedStateConstraint* clone() const override {
                return new ShiftedStateConstraint(std::unique_ptr<StateConstraint>(inner_->clone()), shift_);
            }
            size_t getNumConstraints(scalar_t time) const override { return inner_->getNumConstraints(time); }
            vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& pre) const override {
                return inner_->getValue(time, state, pre).array() - shift_;
            }
            VectorFunctionLinearApproximation getLinearApproximation(
                scalar_t time, const vector_t& state, const PreComputation& pre) const override {
                auto approx = inner_->getLinearApproximation(time, state, pre);
                approx.f.array() -= shift_;
                return approx;
            }
        private:
            std::unique_ptr<StateConstraint> inner_;
            scalar_t shift_{0.0};
        };

        std::unique_ptr<StateConstraint> constraint = std::make_unique<Joint67CouplingConstraint>(
            manipulatorModelInfo_.stateDim, manipulatorModelInfo_.armDim, smoothAbsEps);
        if (margin > 0.0) {
            constraint = std::make_unique<ShiftedStateConstraint>(std::move(constraint), margin);
        }

        ThresholdRelaxedBarrierPenalty::Config barrierConfig{mu, delta, activationThreshold};
        return std::make_unique<StateSoftConstraint>(
            std::move(constraint),
            std::make_unique<ThresholdRelaxedBarrierPenalty>(barrierConfig));
    }

    std::unique_ptr<StateAugmentedLagrangian> MobileManipulatorInterface::getJoint67CouplingAugmentedLagrangian(
        const std::string& taskFile)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        scalar_t scale = 10.0;
        scalar_t stepSize = 1.0;
        scalar_t smoothAbsEps = 1e-6;

        loadData::loadPtreeValue(pt, scale, "joint67Coupling.alScale", false);
        loadData::loadPtreeValue(pt, stepSize, "joint67Coupling.alStepSize", false);
        loadData::loadPtreeValue(pt, smoothAbsEps, "joint67Coupling.smoothAbsEps", false);

        std::cerr << " #### [Joint67 AL] scale=" << scale << " stepSize=" << stepSize << std::endl;

        auto constraint = std::make_unique<Joint67CouplingConstraint>(
            manipulatorModelInfo_.stateDim, manipulatorModelInfo_.armDim, smoothAbsEps);

        augmented::SlacknessSquaredHingePenalty::Config alConfig{scale, stepSize};
        std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyArray;
        penaltyArray.push_back(std::make_unique<augmented::SlacknessSquaredHingePenalty>(alConfig));

        return std::make_unique<StateAugmentedLagrangian>(std::move(constraint), std::move(penaltyArray));
    }

    std::unique_ptr<PinocchioGeometryInterface> MobileManipulatorInterface::getPinocchioGeometryInterface() const
    {
        if (pinocchioGeometryInterfacePtr_)
        {
            // 返回一个副本，因为原始指针是私有的
            return std::make_unique<PinocchioGeometryInterface>(*pinocchioGeometryInterfacePtr_);
        }
        return nullptr;
    }
} // namespace ocs2::mobile_manipulator
