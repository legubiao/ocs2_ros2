#include <pinocchio/fwd.hpp>

#include "ocs2_mobile_manipulator/constraint/EnvironmentCollisionConstraint.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"

#include <pinocchio/algorithm/jacobian.hpp>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2::mobile_manipulator {

EnvironmentCollisionConstraint::EnvironmentCollisionConstraint(
    const PinocchioStateInputMapping<scalar_t>& mapping,
    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
    scalar_t minimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      envGeomInterface_(std::move(envGeomInterface)),
      minimumDistance_(minimumDistance),
      mappingPtr_(mapping.clone())
{
}

EnvironmentCollisionConstraint::EnvironmentCollisionConstraint(
    const EnvironmentCollisionConstraint& rhs)
    : StateConstraint(rhs),
      envGeomInterface_(rhs.envGeomInterface_),
      minimumDistance_(rhs.minimumDistance_),
      mappingPtr_(rhs.mappingPtr_->clone())
{
}

size_t EnvironmentCollisionConstraint::getNumConstraints(scalar_t time) const {
    return envGeomInterface_->getNumCollisionPairs();
}

vector_t EnvironmentCollisionConstraint::getValue(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const
{
    const size_t numPairs = envGeomInterface_->getNumCollisionPairs();
    if (numPairs == 0) {
        return vector_t(0);
    }

    const auto& pinocchioInterface = getPinocchioInterface(preComputation);
    const auto distanceResults = envGeomInterface_->computeDistances(pinocchioInterface);

    vector_t constraints(distanceResults.size());
    for (size_t i = 0; i < distanceResults.size(); ++i) {
        // Use per-obstacle minimumDistance if set, otherwise use default
        const scalar_t minDist = distanceResults[i].minimumDistance > 0.0
            ? distanceResults[i].minimumDistance : minimumDistance_;
        // h = distance - minimumDistance, h > 0 means safe
        constraints[i] = distanceResults[i].distance - minDist;
    }

    return constraints;
}

VectorFunctionLinearApproximation EnvironmentCollisionConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const
{
    const size_t numPairs = envGeomInterface_->getNumCollisionPairs();
    if (numPairs == 0) {
        VectorFunctionLinearApproximation result;
        result.f = vector_t(0);
        result.dfdx = matrix_t(0, state.size());
        return result;
    }

    const auto& pinocchioInterface = getPinocchioInterface(preComputation);
    mappingPtr_->setPinocchioInterface(pinocchioInterface);

    const auto& model = pinocchioInterface.getModel();
    const auto& data = pinocchioInterface.getData();
    const auto& geomModel = envGeomInterface_->getRobotGeometryModel();

    const auto distanceResults = envGeomInterface_->computeDistances(pinocchioInterface);

    VectorFunctionLinearApproximation constraint;
    constraint.f = vector_t(distanceResults.size());
    matrix_t dfdq = matrix_t::Zero(distanceResults.size(), model.nq);

    for (size_t i = 0; i < distanceResults.size(); ++i) {
        const auto& result = distanceResults[i];

        // Use per-obstacle minimumDistance if set, otherwise use default
        const scalar_t minDist = result.minimumDistance > 0.0
            ? result.minimumDistance : minimumDistance_;

        // Constraint value
        constraint.f[i] = result.distance - minDist;

        // Jacobian calculation (similar to SelfCollision)
        const auto jointIdx = result.robotJointIndex;

        // Get joint Jacobian and translate to nearest point
        const Eigen::Vector3d jointPos = data.oMi[jointIdx].translation();
        const Eigen::Vector3d pointOffset = result.nearestPointRobot - jointPos;

        matrix_t jointJacobian = matrix_t::Zero(6, model.nv);
        pinocchio::getJointJacobian(model, data, jointIdx,
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jointJacobian);

        // Point Jacobian = translation Jacobian - skew(offset) * rotation Jacobian
        const matrix_t pointJacobian = jointJacobian.topRows(3) -
            skewSymmetricMatrix(pointOffset) * jointJacobian.bottomRows(3);

        // Distance gradient: direction from robot point to obstacle point
        // When robot moves toward obstacle, distance decreases (negative gradient)
        Eigen::Vector3d distanceGradient;
        if (result.distance > 1e-10) {
            // Normal case: gradient points from robot to obstacle
            distanceGradient = (result.nearestPointObstacle - result.nearestPointRobot).normalized();
        } else {
            // Collision case: gradient points away from obstacle (for escape)
            distanceGradient = (result.nearestPointRobot - result.nearestPointObstacle).normalized();
        }

        // Chain rule: dh/dq = d(distance)/d(point) * d(point)/dq
        // Note: distance decreases when robot point moves toward obstacle
        // So: dh/dq = -gradient^T * pointJacobian
        dfdq.row(i) = -distanceGradient.transpose() * pointJacobian;
    }

    // Convert to OCS2 state space
    matrix_t dfdv = matrix_t::Zero(dfdq.rows(), dfdq.cols());
    std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);

    return constraint;
}

// ============================================================================
// MobileManipulatorEnvironmentCollisionConstraint
// ============================================================================

MobileManipulatorEnvironmentCollisionConstraint::MobileManipulatorEnvironmentCollisionConstraint(
    const PinocchioStateInputMapping<scalar_t>& mapping,
    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
    scalar_t minimumDistance)
    : EnvironmentCollisionConstraint(mapping, std::move(envGeomInterface), minimumDistance)
{
}

MobileManipulatorEnvironmentCollisionConstraint*
MobileManipulatorEnvironmentCollisionConstraint::clone() const {
    return new MobileManipulatorEnvironmentCollisionConstraint(*this);
}

const PinocchioInterface& MobileManipulatorEnvironmentCollisionConstraint::getPinocchioInterface(
    const PreComputation& preComputation) const
{
    return cast<MobileManipulatorPreComputation>(preComputation).getPinocchioInterface();
}

} // namespace ocs2::mobile_manipulator
