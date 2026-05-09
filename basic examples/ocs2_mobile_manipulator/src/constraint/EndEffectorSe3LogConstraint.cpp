/******************************************************************************
 * SE(3) log-map end-effector tracking constraint.
 ******************************************************************************/

#include "ocs2_mobile_manipulator/constraint/EndEffectorSe3LogConstraint.h"

#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"

namespace ocs2::mobile_manipulator
{
    namespace
    {
        inline Eigen::Quaterniond quatFromXyzw(const vector_t& y, int off)
        {
            // TargetTrajectories stores quaternion as (qx,qy,qz,qw) consistent with Eigen::Quaternion::coeffs().
            Eigen::Quaterniond q;
            q.coeffs() = y.segment<4>(off).template cast<double>();
            return q.normalized();
        }

        inline pinocchio::SE3 interpolatePose(const pinocchio::SE3& T0, const pinocchio::SE3& T1, scalar_t alpha)
        {
            const auto p = (scalar_t(1.0) - alpha) * T0.translation() + alpha * T1.translation();
            // Use quaternion slerp for better continuity than log3 near 180deg.
            Eigen::Quaterniond q0(T0.rotation());
            Eigen::Quaterniond q1(T1.rotation());
            if (q0.dot(q1) < 0.0)
            {
                q1.coeffs() *= -1.0; // shortest path
            }
            const Eigen::Quaterniond q = q0.slerp(alpha, q1).normalized();
            const auto R = q.toRotationMatrix();
            return pinocchio::SE3(R, p);
        }

        inline pinocchio::SE3 getReferencePoseAtTime(const TargetTrajectories& target, scalar_t t, bool dualArm,
                                                     int armIdx)
        {
            if (target.timeTrajectory.empty())
            {
                throw std::runtime_error("[EndEffectorSe3LogConstraint] Empty target time trajectory.");
            }

            // Clamp outside range
            if (t <= target.timeTrajectory.front())
            {
                const auto& y = target.stateTrajectory.front();
                const int off = dualArm ? (armIdx * 7) : 0;
                const Eigen::Vector3d p = y.segment<3>(off).template cast<double>();
                const Eigen::Quaterniond q = quatFromXyzw(y, off + 3);
                return pinocchio::SE3(q.toRotationMatrix(), p);
            }
            if (t >= target.timeTrajectory.back())
            {
                const auto& y = target.stateTrajectory.back();
                const int off = dualArm ? (armIdx * 7) : 0;
                const Eigen::Vector3d p = y.segment<3>(off).template cast<double>();
                const Eigen::Quaterniond q = quatFromXyzw(y, off + 3);
                return pinocchio::SE3(q.toRotationMatrix(), p);
            }

            // Find interval (linear scan; trajectories are short in practice)
            size_t k = 0;
            while (k + 1 < target.timeTrajectory.size() && !(target.timeTrajectory[k] <= t && t <= target.timeTrajectory[k + 1]))
            {
                ++k;
            }
            const scalar_t t0 = target.timeTrajectory[k];
            const scalar_t t1 = target.timeTrajectory[k + 1];
            const scalar_t alpha = (t - t0) / (t1 - t0);

            const auto& y0 = target.stateTrajectory[k];
            const auto& y1 = target.stateTrajectory[k + 1];
            const int off = dualArm ? (armIdx * 7) : 0;
            const Eigen::Vector3d p0 = y0.segment<3>(off).template cast<double>();
            const Eigen::Vector3d p1 = y1.segment<3>(off).template cast<double>();
            const Eigen::Quaterniond q0 = quatFromXyzw(y0, off + 3);
            const Eigen::Quaterniond q1 = quatFromXyzw(y1, off + 3);

            const pinocchio::SE3 T0(q0.normalized().toRotationMatrix(), p0);
            const pinocchio::SE3 T1(q1.normalized().toRotationMatrix(), p1);
            return interpolatePose(T0, T1, alpha);
        }
    } // namespace

    EndEffectorSe3LogConstraint::EndEffectorSe3LogConstraint(std::vector<std::string> endEffectorFrames,
                                                             ReferenceManager& referenceManager,
                                                             bool isDualArmMode,
                                                             InvariantType invariantType)
        : StateConstraint(ConstraintOrder::Linear),
          endEffectorFrames_(std::move(endEffectorFrames)),
          referenceManagerPtr_(&referenceManager),
          isDualArmMode_(isDualArmMode),
          invariantType_(invariantType)
    {
        if (referenceManagerPtr_ == nullptr)
        {
            throw std::runtime_error("[EndEffectorSe3LogConstraint] referenceManager is null.");
        }
    }

    size_t EndEffectorSe3LogConstraint::getNumConstraints(scalar_t /*time*/) const
    {
        const size_t nFrames = isDualArmMode_ ? 2 : 1;
        return 6 * nFrames;
    }

    vector_t EndEffectorSe3LogConstraint::getValue(scalar_t time, const vector_t& state,
                                                   const PreComputation& preComputation) const
    {
        const auto& mmPreComp = dynamic_cast<const MobileManipulatorPreComputation&>(preComputation);
        auto& pinocchioInterface = const_cast<PinocchioInterface&>(mmPreComp.getPinocchioInterface());
        const auto& model = pinocchioInterface.getModel();
        auto& data = pinocchioInterface.getData();

        // Lazily resolve frame IDs
        if (frameIds_.empty())
        {
            frameIds_.reserve(endEffectorFrames_.size());
            for (const auto& f : endEffectorFrames_)
            {
                frameIds_.push_back(model.getFrameId(f));
            }
        }

        // Update kinematics
        const auto q = mmPreComp.getPinocchioMapping().getPinocchioJointPosition(state);
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        const auto& target = referenceManagerPtr_->getTargetTrajectories();
        const size_t nFrames = isDualArmMode_ ? 2 : 1;
        vector_t e = vector_t::Zero(static_cast<int>(6 * nFrames));

        for (size_t k = 0; k < nFrames; ++k)
        {
            const pinocchio::SE3 T_ref = getReferencePoseAtTime(target, time, isDualArmMode_, static_cast<int>(k));
            const pinocchio::SE3 T_cur = data.oMf[frameIds_[k]];
            const pinocchio::SE3 T_err =
                (invariantType_ == InvariantType::Right) ? (T_cur.inverse() * T_ref) : (T_ref * T_cur.inverse());
            const auto xi = pinocchio::log6(T_err).toVector();
            e.segment<6>(static_cast<int>(6 * k)) = xi.template cast<scalar_t>();
        }
        return e;
    }

    VectorFunctionLinearApproximation EndEffectorSe3LogConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state, const PreComputation& preComputation) const
    {
        const auto& mmPreComp = dynamic_cast<const MobileManipulatorPreComputation&>(preComputation);
        auto& pinocchioInterface = const_cast<PinocchioInterface&>(mmPreComp.getPinocchioInterface());
        const auto& model = pinocchioInterface.getModel();
        auto& data = pinocchioInterface.getData();

        if (frameIds_.empty())
        {
            frameIds_.reserve(endEffectorFrames_.size());
            for (const auto& f : endEffectorFrames_)
            {
                frameIds_.push_back(model.getFrameId(f));
            }
        }

        VectorFunctionLinearApproximation approx(static_cast<int>(getNumConstraints(time)), state.size());
        approx.f = getValue(time, state, preComputation);

        const auto q = mmPreComp.getPinocchioMapping().getPinocchioJointPosition(state);
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        const auto& target = referenceManagerPtr_->getTargetTrajectories();
        const size_t nFrames = isDualArmMode_ ? 2 : 1;

        for (size_t k = 0; k < nFrames; ++k)
        {
            const pinocchio::SE3 T_ref = getReferencePoseAtTime(target, time, isDualArmMode_, static_cast<int>(k));
            const pinocchio::SE3 T_cur = data.oMf[frameIds_[k]];
            const pinocchio::SE3 T_err =
                (invariantType_ == InvariantType::Right) ? (T_cur.inverse() * T_ref) : (T_ref * T_cur.inverse());

            // Frame Jacobian in LOCAL frame
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> J(6, model.nv);
            J.setZero();
            pinocchio::computeFrameJacobian(model, data, q, frameIds_[k], pinocchio::ReferenceFrame::LOCAL, J);

            const Eigen::Matrix<scalar_t, 6, 6> Jlog = pinocchio::Jlog6(T_err).template cast<scalar_t>();
            // d(log(T_err))/dq approx: -Jlog * J (matches right-invariant choice; acceptable in practice here)
            const auto Jq = -Jlog * J;
            const matrix_t Jv = matrix_t::Zero(Jq.rows(), Jq.cols());
            const auto [dfdx, /*dfdu*/ _] = mmPreComp.getPinocchioMapping().getOcs2Jacobian(state, Jq, Jv);
            approx.dfdx.block(static_cast<int>(6 * k), 0, 6, state.size()) = dfdx;
        }

        return approx;
    }
} // namespace ocs2::mobile_manipulator

