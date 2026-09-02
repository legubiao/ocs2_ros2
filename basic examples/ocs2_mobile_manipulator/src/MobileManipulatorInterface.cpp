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
#include <optional>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/constraint/BodyRelativeConstraint.h"
#include "ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h"

#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/constraint/EnvironmentCollisionConstraint.h"
#include "ocs2_mobile_manipulator/cost/QuadraticInputCost.h"
#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/FloatingArmManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/WheelBasedMobileManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/OmniWheelBasedMobileManipulatorDynamics.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


namespace ocs2::mobile_manipulator
{
    MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile,
                                                           const std::string& libraryFolder,
                                                           const std::string& urdfFile,
                                                           const FrameOverrides& frameOverrides)
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

        // Optional in-memory overrides (e.g. ROS controller params); empty = keep info value.
        if (!frameOverrides.baseFrame.empty())
        {
            baseFrame = frameOverrides.baseFrame;
        }
        if (!frameOverrides.eeFrame.empty())
        {
            eeFrame = frameOverrides.eeFrame;
        }
        if (!frameOverrides.eeFrame1.empty())
        {
            eeFrame1 = frameOverrides.eeFrame1;
        }

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
        std::cerr << "\n #### model_information.eeFrame (left): \"" << eeFrame << "\"";
        std::cerr << "\n #### model_information.eeFrame1 (right): \"" << eeFrame1 << "\"" << std::endl;
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

        // SQP settings (optional, will use defaults if not present)
        try {
            sqpSettings_ = sqp::loadSettings(taskFile, "sqp");
        } catch (const std::exception& e) {
            std::cerr << " #### SQP settings not found in task file, using defaults.\n";
        }

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

        // 6/7-axis coupling constraint (Marvin CCS coupled wrist range).
        // Parse the arms regardless of activation so runtime detection (checkJoint67Coupling)
        // works even when the soft constraint itself is disabled.
        bool activateJoint67Coupling = false;
        loadData::loadPtreeValue(pt, activateJoint67Coupling, "joint67Coupling.activate", false);
        if (pt.get_child_optional("joint67Coupling"))
        {
            auto couplingCost = getJoint67CouplingConstraint(taskFile);
            if (activateJoint67Coupling)
            {
                problem_.stateSoftConstraintPtr->add("joint67Coupling", std::move(couplingCost));
            }
        }

        // environment collision avoidance constraint
        envCollisionEnabled_ = false;
        loadData::loadPtreeValue(pt, envCollisionEnabled_, "environmentCollision.activate", false);
        if (envCollisionEnabled_)
        {
            problem_.stateSoftConstraintPtr->add(
                "environmentCollision", getEnvironmentCollisionConstraint(*pinocchioInterfacePtr_, taskFile,
                                                                          "environmentCollision"));
        }

        // Dynamics
        switch (manipulatorModelInfo_.manipulatorModelType)
        {
        case ManipulatorModelType::DefaultManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<DefaultManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", libraryFolder,
                    recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::FloatingArmManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<FloatingArmManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", libraryFolder,
                    recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<FullyActuatedFloatingArmManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics",
                    libraryFolder, recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::WheelBasedMobileManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<WheelBasedMobileManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", libraryFolder,
                    recompileLibraries, true);
                break;
            }
        case ManipulatorModelType::OmniWheelBasedMobileManipulator:
            {
                problem_.dynamicsPtr = std::make_unique<OmniWheelBasedMobileManipulatorDynamics>(
                    manipulatorModelInfo_, "dynamics", libraryFolder,
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

        std::cerr << " #### Dual arm mode: " << (dual_arm_ ? "enabled" : "disabled") << std::endl;
        std::cerr << " #### =============================================================================\n";

        if (referenceManagerPtr_ == nullptr)
        {
            throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
        }

        std::unique_ptr<StateConstraint> constraint;
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
                constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, true);
            }
            else
            {
                PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,
                                                            {manipulatorModelInfo_.eeFrame});
                constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, false);
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
                constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, true);
            }
            else
            {
                PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd,
                                                                 {manipulatorModelInfo_.eeFrame},
                                                                 manipulatorModelInfo_.stateDim,
                                                                 manipulatorModelInfo_.inputDim,
                                                                 "end_effector_kinematics", libraryFolder,
                                                                 recompileLibraries, false);
                constraint = std::make_unique<EndEffectorConstraint>(eeKinematics, *referenceManagerPtr_, false);
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

            loadData::loadPtreeValue(pt, leftMuPosition, prefix + ".leftArm.muPosition", false);
            loadData::loadPtreeValue(pt, leftMuOrientation, prefix + ".leftArm.muOrientation", false);
            loadData::loadPtreeValue(pt, rightMuPosition, prefix + ".rightArm.muPosition", false);
            loadData::loadPtreeValue(pt, rightMuOrientation, prefix + ".rightArm.muOrientation", false);

            penaltyArray.resize(12);
            // Left arm: position + orientation
            std::generate_n(penaltyArray.begin(), 3, [&]
            {
                return std::make_unique<QuadraticPenalty>(leftMuPosition);
            });
            std::generate_n(penaltyArray.begin() + 3, 3, [&]
            {
                return std::make_unique<QuadraticPenalty>(leftMuOrientation);
            });
            // Right arm: position + orientation
            std::generate_n(penaltyArray.begin() + 6, 3, [&]
            {
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
            std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
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

        // Create geometry interface (also stored for environment collision to reuse)
        pinocchioGeometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(
            pinocchioInterface, urdfFile, collisionLinkPairs, collisionObjectPairs);

        const size_t numCollisionPairs = pinocchioGeometryInterfacePtr_->getNumCollisionPairs();
        std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

        std::unique_ptr<StateConstraint> constraint;
        if (usePreComputation)
        {
            PinocchioGeometryInterface geometryInterfaceCopy(*pinocchioGeometryInterfacePtr_);
            constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(
                MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                std::move(geometryInterfaceCopy), minimumDistance);
        }
        else
        {
            PinocchioGeometryInterface geometryInterfaceCopy(*pinocchioGeometryInterfacePtr_);
            constraint = std::make_unique<SelfCollisionConstraintCppAd>(
                pinocchioInterface, MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                std::move(geometryInterfaceCopy), minimumDistance,
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

            // arm joint DOF limits from the parsed URDF
            const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
            const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

            std::cerr << "\n #### JointPositionLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
            std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
            loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
            std::cerr << " #### =============================================================================\n";

            stateLimits.reserve(armStateDim);
            for (int i = 0; i < armStateDim; ++i)
            {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index = baseStateDim + i;
                boxConstraint.lowerBound = lowerBound(i);
                boxConstraint.upperBound = upperBound(i);
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

    std::unique_ptr<StateCost> MobileManipulatorInterface::getJoint67CouplingConstraint(
        const std::string& taskFile)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        const std::string prefix = "joint67Coupling";
        const size_t baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;

        // 6/7-axis coupling is a fixed property of the CCS wrist: it always acts on the
        // 6th/7th joint of each 7-DOF arm, with fixed BD coefficients (from MvKDCfg).
        // Barrier weight is configurable in the task .info file (joint67Coupling.mu).
        constexpr scalar_t kDefaultMu = 1e-3;
        constexpr scalar_t kDefaultDelta = 1e-3;
        constexpr scalar_t kDeadbandDeg = 1.0;

        scalar_t mu = kDefaultMu;
        scalar_t delta = kDefaultDelta;
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
        const Joint67CouplingConstraint::Parabola kPp{0.018004, -2.3205, 108.0};   // J6>=0, J7 upper
        const Joint67CouplingConstraint::Parabola kNp{0.018004, -2.3205, 108.0};   // J6<0,  J7 upper
        const Joint67CouplingConstraint::Parabola kNn{-0.018004, 2.3205, -108.0};  // J6<0,  J7 lower
        const Joint67CouplingConstraint::Parabola kPn{-0.018004, 2.3205, -108.0};  // J6>=0, J7 lower

        const size_t armDim = static_cast<size_t>(manipulatorModelInfo_.armDim);

        auto makeArm = [&](size_t j6StateIndex, size_t j7StateIndex)
        {
            Joint67CouplingConstraint::ArmCoupling arm;
            arm.j6StateIndex = j6StateIndex;
            arm.j7StateIndex = j7StateIndex;
            arm.deadbandDeg = kDeadbandDeg;
            arm.pp = kPp;
            arm.np = kNp;
            arm.nn = kNn;
            arm.pn = kPn;
            return arm;
        };

        // Optional per-arm J6/J7 index override (only needed for non-standard layouts,
        // e.g. wheel humanoid with waist/head joints before the arms).
        auto loadArm = [&](const std::string& side, size_t defaultJ6, size_t defaultJ7)
            -> std::optional<Joint67CouplingConstraint::ArmCoupling>
        {
            const std::string armPrefix = prefix + "." + side;
            if (!pt.get_child_optional(armPrefix))
            {
                return std::nullopt;
            }
            size_t j6Index = defaultJ6;
            size_t j7Index = defaultJ7;
            loadData::loadPtreeValue(pt, j6Index, armPrefix + ".j6Index", false);
            loadData::loadPtreeValue(pt, j7Index, armPrefix + ".j7Index", false);
            if (j6Index >= armDim || j7Index >= armDim)
            {
                throw std::runtime_error(
                    "[MobileManipulatorInterface] joint67Coupling." + side +
                    " j6Index/j7Index is outside armDim");
            }
            auto arm = makeArm(baseStateDim + j6Index, baseStateDim + j7Index);
            std::cerr << " ####   " << side << ": J6/J7 state indices "
                      << arm.j6StateIndex << "/" << arm.j7StateIndex << '\n';
            return arm;
        };

        std::vector<Joint67CouplingConstraint::ArmCoupling> arms;
        if (auto left = loadArm("left", 5, 6))
        {
            arms.push_back(*left);
        }
        if (auto right = loadArm("right", 12, 13))
        {
            arms.push_back(*right);
        }
        // Default: coupling always applies to the 6/7 axes of each 7-DOF arm.
        if (arms.empty())
        {
            if (armDim >= 7)
            {
                arms.push_back(makeArm(baseStateDim + 5, baseStateDim + 6));
            }
            if (armDim >= 14)
            {
                arms.push_back(makeArm(baseStateDim + 12, baseStateDim + 13));
            }
        }
        std::cerr << " #### Joint67Coupling: mu=" << mu
                  << " delta=" << delta
                  << " deadbandDeg=" << kDeadbandDeg << " arms=" << arms.size() << '\n';
        std::cerr << " #### =============================================================================\n";

        if (arms.empty())
        {
            throw std::runtime_error(
                "[MobileManipulatorInterface] joint67Coupling enabled but armDim too small "
                "for a 6/7-axis coupling layout");
        }

        // Keep a degree-domain copy for runtime checkJoint67Coupling().
        joint67CouplingArms_ = arms;

        auto constraint = std::make_unique<Joint67CouplingConstraint>(
            std::move(arms), manipulatorModelInfo_.stateDim);

        std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(constraint->getNumConstraints(0.0));
        std::generate_n(penaltyArray.begin(), penaltyArray.size(), [&]
        {
            return std::make_unique<RelaxedBarrierPenalty>(
                RelaxedBarrierPenalty::Config{mu, delta});
        });

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
    }

    std::vector<Joint67CouplingConstraint::CouplingStatus> MobileManipulatorInterface::checkJoint67Coupling(
        const vector_t& state) const
    {
        constexpr scalar_t kRadToDeg = 180.0 / 3.14159265358979323846;
        std::vector<Joint67CouplingConstraint::CouplingStatus> statuses;
        statuses.reserve(joint67CouplingArms_.size());
        for (const auto& arm : joint67CouplingArms_)
        {
            const scalar_t j6Deg = state(static_cast<Eigen::Index>(arm.j6StateIndex)) * kRadToDeg;
            const scalar_t j7Deg = state(static_cast<Eigen::Index>(arm.j7StateIndex)) * kRadToDeg;
            statuses.push_back(Joint67CouplingConstraint::checkCouplingDeg(
                j6Deg, j7Deg, arm.pp, arm.np, arm.nn, arm.pn, arm.deadbandDeg));
        }
        return statuses;
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

    std::unique_ptr<PinocchioGeometryInterface> MobileManipulatorInterface::getPinocchioGeometryInterface() const
    {
        if (pinocchioGeometryInterfacePtr_)
        {
            // 返回一个副本，因为原始指针是私有的
            return std::make_unique<PinocchioGeometryInterface>(*pinocchioGeometryInterfacePtr_);
        }
        return nullptr;
    }

    std::unique_ptr<StateCost> MobileManipulatorInterface::getEnvironmentCollisionConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& prefix)
    {
        std::vector<std::string> collisionLinks;
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t minimumDistance = 0.0;
        scalar_t activationDistance = -1.0;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### EnvironmentCollision Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
        loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
        loadData::loadPtreeValue(pt, activationDistance, prefix + ".activationDistance", false);
        loadData::loadStdVector<std::string>(taskFile, prefix + ".collisionLinks", collisionLinks, true);

        // If activationDistance not specified, default to 5 * minimumDistance
        if (activationDistance < 0.0) {
            activationDistance = 5.0 * minimumDistance;
        }

        // Store distances for later use
        envCollisionMinimumDistance_ = minimumDistance;
        envCollisionActivationDistance_ = activationDistance;

        std::cerr << " #### minimumDistance: " << minimumDistance << " (minimum allowed distance)\n";
        std::cerr << " #### activationDistance: " << activationDistance << " (penalty only active when distance < this value)\n";
        std::cerr << " #### collisionLinks: [";
        for (size_t i = 0; i < collisionLinks.size(); ++i) {
            std::cerr << collisionLinks[i];
            if (i < collisionLinks.size() - 1) std::cerr << ", ";
        }
        std::cerr << "]\n";
        std::cerr << " #### =============================================================================\n";

        // Environment collision requires self-collision to be enabled (for robot geometry model)
        if (!pinocchioGeometryInterfacePtr_) {
            throw std::runtime_error(
                "[EnvironmentCollision] Environment collision requires selfCollision to be enabled first!");
        }
        
        // Create environment geometry interface (reuses geometry model from self-collision)
        envGeomInterfacePtr_ = std::make_shared<EnvironmentGeometryInterface>(
            *pinocchioGeometryInterfacePtr_, pinocchioInterface, collisionLinks);

        // Load initial obstacles from config
        loadInitialObstacles(taskFile, prefix);

        // Create the constraint
        auto constraint = std::make_unique<MobileManipulatorEnvironmentCollisionConstraint>(
            MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
            envGeomInterfacePtr_,
            minimumDistance);

        // Use ThresholdRelaxedBarrierPenalty with activation distance
        const scalar_t activationThreshold = activationDistance - minimumDistance;
        auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
            ThresholdRelaxedBarrierPenalty::Config{mu, delta, activationThreshold});

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }


    void MobileManipulatorInterface::loadInitialObstacles(const std::string& taskFile, const std::string& prefix)
    {
        if (!envGeomInterfacePtr_) {
            return;
        }

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        // Try to get the obstacles subtree
        const std::string obstaclesKey = prefix + ".obstacles";
        auto obstaclesOpt = pt.get_child_optional(obstaclesKey);
        if (!obstaclesOpt) {
            std::cerr << " #### No initial obstacles configured.\n";
            return;
        }

        std::cerr << " #### Loading initial obstacles:\n";
        int obstacleCount = 0;

        for (const auto& obstaclePair : obstaclesOpt.get()) {
            const std::string& obstacleName = obstaclePair.first;
            const auto& obstacleNode = obstaclePair.second;

            // Get obstacle type
            std::string type = obstacleNode.get<std::string>("type", "");
            if (type.empty()) {
                std::cerr << " ####   Warning: obstacle '" << obstacleName << "' has no type, skipping.\n";
                continue;
            }

            // Get position (required)
            vector_t position = vector_t::Zero(3);
            auto posOpt = obstacleNode.get_child_optional("position");
            if (posOpt) {
                int idx = 0;
                for (const auto& val : posOpt.get()) {
                    if (idx < 3) position(idx++) = std::stod(val.second.data());
                }
            }

            // Get orientation (optional, default identity)
            Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
            auto orientOpt = obstacleNode.get_child_optional("orientation");
            if (orientOpt) {
                std::vector<double> quat;
                for (const auto& val : orientOpt.get()) {
                    quat.push_back(std::stod(val.second.data()));
                }
                if (quat.size() == 4) {
                    orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);  // w, x, y, z
                }
            }

            // Get per-obstacle minimumDistance (optional, 0 uses default)
            double obsMinDist = obstacleNode.get<double>("minimumDistance", 0.0);

            if (type == "box") {
                vector_t halfExtents = vector_t::Zero(3);
                auto sizeOpt = obstacleNode.get_child_optional("halfExtents");
                if (sizeOpt) {
                    int idx = 0;
                    for (const auto& val : sizeOpt.get()) {
                        if (idx < 3) halfExtents(idx++) = std::stod(val.second.data());
                    }
                }
                envGeomInterfacePtr_->addBox(obstacleName, halfExtents, position, orientation, obsMinDist);
                std::cerr << " ####   - Box '" << obstacleName << "': halfExtents=(" << halfExtents.transpose() 
                          << "), pos=(" << position.transpose() << "), minDist=" << obsMinDist << "\n";
            }
            else if (type == "sphere") {
                double radius = obstacleNode.get<double>("radius", 0.1);
                envGeomInterfacePtr_->addSphere(obstacleName, radius, position, obsMinDist);
                std::cerr << " ####   - Sphere '" << obstacleName << "': radius=" << radius 
                          << ", pos=(" << position.transpose() << "), minDist=" << obsMinDist << "\n";
            }
            else if (type == "cylinder") {
                double radius = obstacleNode.get<double>("radius", 0.1);
                double height = obstacleNode.get<double>("height", 0.2);
                envGeomInterfacePtr_->addCylinder(obstacleName, radius, height, position, orientation, obsMinDist);
                std::cerr << " ####   - Cylinder '" << obstacleName << "': radius=" << radius 
                          << ", height=" << height << ", pos=(" << position.transpose() << "), minDist=" << obsMinDist << "\n";
            }
            else {
                std::cerr << " ####   Warning: unknown obstacle type '" << type << "' for '" << obstacleName << "'\n";
                continue;
            }
            obstacleCount++;
        }
        std::cerr << " #### Loaded " << obstacleCount << " initial obstacles.\n";
    }
} // namespace ocs2::mobile_manipulator
