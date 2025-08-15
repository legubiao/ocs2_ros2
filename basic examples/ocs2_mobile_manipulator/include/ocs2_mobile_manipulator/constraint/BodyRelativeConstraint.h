#pragma once

#include <memory>
#include <string>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2::mobile_manipulator
{
    class BodyRelativeConstraint final : public StateConstraint
    {
    public:
        using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
        using quaternion_t = Eigen::Quaternion<scalar_t>;
        using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;

        BodyRelativeConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                               const std::string& bodyLinkName,
                               scalar_t rollTolerance,
                               scalar_t pitchTolerance,
                               scalar_t muRoll,
                               scalar_t muPitch,
                               scalar_t muPositionX,
                               scalar_t muPositionY);

        ~BodyRelativeConstraint() override = default;

        BodyRelativeConstraint* clone() const override
        {
            return new BodyRelativeConstraint(*endEffectorKinematicsPtr_, bodyLinkName_,
                                              rollTolerance_, pitchTolerance_,
                                              muRoll_, muPitch_,
                                              muPositionX_, muPositionY_);
        }

        size_t getNumConstraints(scalar_t time) const override;
        vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComputation) const override;

    private:
        BodyRelativeConstraint(const BodyRelativeConstraint& other) = default;

        // Calculate attitude constraints relative to base frame
        vector_t computeOrientationConstraints(const quaternion_t& bodyOrientation) const;


        // Constraint parameters
        std::string bodyLinkName_;
        scalar_t rollTolerance_;
        scalar_t pitchTolerance_;
        scalar_t muRoll_;
        scalar_t muPitch_;
        scalar_t muPositionX_;
        scalar_t muPositionY_;

        // Kinematics interface
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
        PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

        // Cached target orientation (relative to base frame)
        quaternion_t targetOrientation_;
        vector3_t targetPosition_;
    };
}
