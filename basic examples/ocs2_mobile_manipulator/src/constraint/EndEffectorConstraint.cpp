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

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h>

#include <ocs2_core/misc/LinearInterpolation.h>


namespace ocs2::mobile_manipulator
{
    EndEffectorConstraint::EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                 const ReferenceManager& referenceManager,
                                                 bool dualArmMode)
        : StateConstraint(ConstraintOrder::Linear),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          referenceManagerPtr_(&referenceManager),
          dualArmMode_(dualArmMode)
    {
        if (dualArmMode_) {
            // Dual-arm mode: check if there are exactly 2 end effectors
            if (endEffectorKinematics.getIds().size() != 2)
            {
                throw std::runtime_error(
                    "[EndEffectorConstraint] Dual arm mode requires exactly 2 end effector IDs, got " + 
                    std::to_string(endEffectorKinematics.getIds().size()));
            }
        } else {
            if (endEffectorKinematics.getIds().size() != 1)
            {
                throw std::runtime_error(
                    "[EndEffectorConstraint] Single arm mode requires exactly 1 end effector ID, got " + 
                    std::to_string(endEffectorKinematics.getIds().size()));
            }
        }
        pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
    }


    size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const
    {
        return dualArmMode_ ? 12 : 6;  // Dual-arm: 12D, Single-arm: 6D
    }


    vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state,
                                             const PreComputation& preComputation) const
    {
        // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
        if (pinocchioEEKinPtr_ != nullptr)
        {
            const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
            pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
        }

        if (dualArmMode_) {
            // Dual-arm mode: calculate constraints for left and right arms
            const auto leftArmPose = interpolateLeftArmPose(time);
            const auto rightArmPose = interpolateRightArmPose(time);
            
            const auto positions = endEffectorKinematicsPtr_->getPosition(state);
            const auto orientationErrors = endEffectorKinematicsPtr_->getOrientationError(state, 
                {leftArmPose.second, rightArmPose.second});
            
            vector_t constraint(12);
            // Left arm constraints: position + orientation
            constraint.head<3>() = positions[0] - leftArmPose.first;
            constraint.segment<3>(3) = orientationErrors[0];
            // Right arm constraints: position + orientation
            constraint.segment<3>(6) = positions[1] - rightArmPose.first;
            constraint.tail<3>() = orientationErrors[1];
            
            return constraint;
        } else {
            const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

            vector_t constraint(6);
            constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;
            constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
            return constraint;
        }
    }


    VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const
    {
        // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
        if (pinocchioEEKinPtr_ != nullptr)
        {
            const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
            pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
        }

        if (dualArmMode_) {
            // Dual-arm mode: calculate linear approximation for left and right arms
            const auto leftArmPose = interpolateLeftArmPose(time);
            const auto rightArmPose = interpolateRightArmPose(time);
            
            const auto positions = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);
            const auto orientationErrors = endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, 
                {leftArmPose.second, rightArmPose.second});
            
            auto approximation = VectorFunctionLinearApproximation(12, state.rows(), 0);
            
            // Left arm linear approximation
            approximation.f.head<3>() = positions[0].f - leftArmPose.first;
            approximation.dfdx.topRows<3>() = positions[0].dfdx;
            approximation.f.segment<3>(3) = orientationErrors[0].f;
            approximation.dfdx.middleRows<3>(3) = orientationErrors[0].dfdx;
            
            // Right arm linear approximation
            approximation.f.segment<3>(6) = positions[1].f - rightArmPose.first;
            approximation.dfdx.middleRows<3>(6) = positions[1].dfdx;
            approximation.f.tail<3>() = orientationErrors[1].f;
            approximation.dfdx.bottomRows<3>() = orientationErrors[1].dfdx;
            
            return approximation;
        } else {
            const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

            auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

            const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
            approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
            approximation.dfdx.topRows<3>() = eePosition.dfdx;

            const auto eeOrientationError =
                endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
            approximation.f.tail<3>() = eeOrientationError.f;
            approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;

            return approximation;
        }
    }


    auto EndEffectorConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t>
    {
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;

        vector_t position;
        quaternion_t orientation;

        if (stateTrajectory.size() > 1)
        {
            // Normal interpolation case
            int index;
            scalar_t alpha;
            std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

            const auto& lhs = stateTrajectory[index];
            const auto& rhs = stateTrajectory[index + 1];
            const quaternion_t q_lhs(lhs.tail<4>());
            const quaternion_t q_rhs(rhs.tail<4>());

            position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
            orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
        }
        else
        {
            // stateTrajectory.size() == 1
            position = stateTrajectory.front().head<3>();
            orientation = quaternion_t(stateTrajectory.front().tail<4>());
        }

        return {position, orientation};
    }

    auto EndEffectorConstraint::interpolateLeftArmPose(scalar_t time) const -> std::pair<vector_t, quaternion_t>
    {
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;

        vector_t position;
        quaternion_t orientation;

        if (stateTrajectory.size() > 1)
        {
            // Normal interpolation case
            int index;
            scalar_t alpha;
            std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

            const auto& lhs = stateTrajectory[index];
            const auto& rhs = stateTrajectory[index + 1];
            
            // Dual-arm mode: left arm in first 7 dimensions [left_x, left_y, left_z, left_qw, left_qx, left_qy, left_qz]
            const quaternion_t q_lhs(lhs.segment<4>(3));
            const quaternion_t q_rhs(rhs.segment<4>(3));

            position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
            orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
        }
        else
        {
            // stateTrajectory.size() == 1
            position = stateTrajectory.front().head<3>();
            orientation = quaternion_t(stateTrajectory.front().segment<4>(3));
        }

        return {position, orientation};
    }

    auto EndEffectorConstraint::interpolateRightArmPose(scalar_t time) const -> std::pair<vector_t, quaternion_t>
    {
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;

        vector_t position;
        quaternion_t orientation;

        if (stateTrajectory.size() > 1)
        {
            // Normal interpolation case
            int index;
            scalar_t alpha;
            std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

            const auto& lhs = stateTrajectory[index];
            const auto& rhs = stateTrajectory[index + 1];
            
            // Dual-arm mode: right arm in last 7 dimensions [right_x, right_y, right_z, right_qw, right_qx, right_qy, right_qz]
            const quaternion_t q_lhs(lhs.segment<4>(10));
            const quaternion_t q_rhs(rhs.segment<4>(10));

            position = alpha * lhs.segment<3>(7) + (1.0 - alpha) * rhs.segment<3>(7);
            orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
        }
        else
        {
            // stateTrajectory.size() == 1
            position = stateTrajectory.front().segment<3>(7);
            orientation = quaternion_t(stateTrajectory.front().segment<4>(10));
        }

        return {position, orientation};
    }
} // namespace ocs2::mobile_manipulator
