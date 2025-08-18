#include "ocs2_mobile_manipulator/constraint/BodyRelativeConstraint.h"

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/constraint/ConstraintOrder.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

namespace ocs2::mobile_manipulator
{
    BodyRelativeConstraint::BodyRelativeConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                   const std::string& bodyLinkName,
                                                   scalar_t rollTolerance,
                                                   scalar_t pitchTolerance,
                                                   int modelType)
        : StateConstraint(ConstraintOrder::Linear)
          , bodyLinkName_(bodyLinkName)
          , rollTolerance_(rollTolerance)
          , pitchTolerance_(pitchTolerance)
          , modelType_(modelType)
          , endEffectorKinematicsPtr_(endEffectorKinematics.clone())
    {
        // 初始化目标姿态为单位四元数（无旋转）
        targetOrientation_ = quaternion_t::Identity();
        targetPosition_ = vector3_t::Zero();

        // Cache PinocchioEndEffectorKinematics pointer
        pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());

        // Verify if bodyLinkName_ exists in the kinematics interface ID list
        const auto& availableIds = endEffectorKinematicsPtr_->getIds();
        bool found = false;
        for (const auto& id : availableIds)
        {
            if (id == bodyLinkName_)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            std::cerr << "[BodyRelativeConstraint] Warning: bodyLinkName '" << bodyLinkName_
                << "' not found in available IDs: ";
            for (const auto& id : availableIds)
            {
                std::cerr << "'" << id << "' ";
            }
            std::cerr << std::endl;
        }
    }

    size_t BodyRelativeConstraint::getNumConstraints(scalar_t time) const
    {
        // Attitude constraints: roll and pitch + Position constraints: x, y
        return 4;
    }

    vector_t BodyRelativeConstraint::getValue(scalar_t time, const vector_t& state,
                                              const PreComputation& preComputation) const
    {
        // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface
        if (pinocchioEEKinPtr_ != nullptr)
        {
            const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
            pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
        }

        // Update targetPosition based on model type
        updateTargetPosition(state);

        // Get orientation and position errors
        const auto orientationErrors = endEffectorKinematicsPtr_->getOrientationError(state, {targetOrientation_});
        const auto positions = endEffectorKinematicsPtr_->getPosition(state);

        // Calculate position error (relative to base frame)
        vector3_t positionError = positions[0] - targetPosition_;

        // Calculate constraint values: first 2 are attitude constraints, last 2 are position constraints (X and Y)
        vector_t constraints(getNumConstraints(time));

        // Attitude constraints: roll and pitch
        constraints.head<2>() = orientationErrors[0].head<2>();

        // Position constraints: x, y (Z direction unconstrained, allowing free vertical movement)
        constraints.tail<2>() = positionError.head<2>();

        return constraints;
    }

    VectorFunctionLinearApproximation BodyRelativeConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const
    {
        // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface
        if (pinocchioEEKinPtr_ != nullptr)
        {
            const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
            pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
        }

        // Update targetPosition based on model type (same logic as getValue)
        updateTargetPosition(state);

        // Get linear approximation of orientation and position errors
        const auto orientationErrors = endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(
            state, {targetOrientation_});
        const auto positions = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);

        // Calculate linear approximation of constraints: first 2 are attitude constraints, last 2 are position constraints
        auto approximation = VectorFunctionLinearApproximation(getNumConstraints(time), state.rows(), 0);

        // Attitude constraints: roll and pitch
        approximation.f.head<2>() = orientationErrors[0].f.head<2>();
        approximation.dfdx.topRows<2>() = orientationErrors[0].dfdx.topRows<2>();

        // Position constraints: x, y (Z direction unconstrained, allowing free vertical movement)
        // Position error = current position - target position
        approximation.f.tail<2>() = (positions[0].f - targetPosition_).head<2>();
        approximation.dfdx.bottomRows<2>() = positions[0].dfdx.topRows<2>();

        return approximation;
    }

    void BodyRelativeConstraint::updateTargetPosition(const vector_t& state) const
    {
        if (modelType_ == 1 && endEffectorKinematicsPtr_ != nullptr)
        {
            // For mobile robots (WheelBasedMobileManipulator): 
            // Use the existing EndEffectorKinematics to get the base frame position
            try {
                // Get positions from kinematics interface
                // positions[0] = end effector position (bodyLinkName)
                // positions[1] = base frame position (baseFrame)
                const auto positions = endEffectorKinematicsPtr_->getPosition(state);
                if (positions.size() >= 2) {
                    // The second position should be the base frame position
                    targetPosition_ = positions[1];
                }
            } catch (const std::exception& e) {
                // If EndEffectorKinematics fails, keep targetPosition as zero
                // This ensures the constraint falls back to fixed-base behavior
            }
        }
        // For other robot types (DefaultManipulator, FloatingArmManipulator, etc.):
        // Keep targetPosition as zero (initialized value) - fixed base reference
    }
}
