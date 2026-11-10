#include "ocs2_mobile_manipulator/constraint/HomeJointReferenceConstraint.h"

#include <stdexcept>
#include <utility>

namespace ocs2::mobile_manipulator {

HomeJointReferenceConstraint::HomeJointReferenceConstraint(
    const HomeJointReferenceSource& referenceSource,
    size_t baseStateDim,
    std::vector<size_t> trackedArmIndices)
    : StateConstraint(ConstraintOrder::Linear),
      referenceSourcePtr_(&referenceSource),
      baseStateDim_(baseStateDim),
      trackedArmIndices_(std::move(trackedArmIndices)) {
    if (trackedArmIndices_.empty()) {
        throw std::invalid_argument(
            "[HomeJointReferenceConstraint] trackedArmIndices cannot be empty");
    }
}

bool HomeJointReferenceConstraint::isActive(scalar_t /*time*/) const {
    if (!referenceSourcePtr_->isHomeJointReferenceEnabled()) {
        return false;
    }
    const auto& qHome = referenceSourcePtr_->getHomeJointReference();
    return qHome.size() > 0;
}

size_t HomeJointReferenceConstraint::getNumConstraints(scalar_t time) const {
    return isActive(time) ? trackedArmIndices_.size() : 0;
}

void HomeJointReferenceConstraint::validateStateSize(const vector_t& state) const {
    const auto& qHome = referenceSourcePtr_->getHomeJointReference();
    for (const auto armIndex : trackedArmIndices_) {
        const size_t stateIndex = baseStateDim_ + armIndex;
        if (stateIndex >= static_cast<size_t>(state.size())) {
            throw std::out_of_range(
                "[HomeJointReferenceConstraint] tracked joint is outside state dimension");
        }
        if (armIndex >= static_cast<size_t>(qHome.size())) {
            throw std::out_of_range(
                "[HomeJointReferenceConstraint] tracked joint is outside HOME reference dimension");
        }
    }
}

vector_t HomeJointReferenceConstraint::getValue(
    scalar_t time, const vector_t& state,
    const PreComputation& /*preComputation*/) const {
    if (!isActive(time)) {
        return vector_t();
    }
    validateStateSize(state);

    const auto& qHome = referenceSourcePtr_->getHomeJointReference();
    const Eigen::Index n = static_cast<Eigen::Index>(trackedArmIndices_.size());
    vector_t out(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const size_t armIndex = trackedArmIndices_[static_cast<size_t>(i)];
        const Eigen::Index stateIndex =
            static_cast<Eigen::Index>(baseStateDim_ + armIndex);
        out(i) = state(stateIndex) - qHome(static_cast<Eigen::Index>(armIndex));
    }
    return out;
}

VectorFunctionLinearApproximation HomeJointReferenceConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state,
    const PreComputation& /*preComputation*/) const {
    if (!isActive(time)) {
        return VectorFunctionLinearApproximation(0, state.size(), 0);
    }
    validateStateSize(state);

    const auto& qHome = referenceSourcePtr_->getHomeJointReference();
    const Eigen::Index n = static_cast<Eigen::Index>(trackedArmIndices_.size());
    VectorFunctionLinearApproximation approx(n, state.size(), 0);
    approx.f.setZero();
    approx.dfdx.setZero();

    for (Eigen::Index i = 0; i < n; ++i) {
        const size_t armIndex = trackedArmIndices_[static_cast<size_t>(i)];
        const Eigen::Index stateIndex =
            static_cast<Eigen::Index>(baseStateDim_ + armIndex);
        approx.f(i) = state(stateIndex) - qHome(static_cast<Eigen::Index>(armIndex));
        approx.dfdx(i, stateIndex) = 1.0;
    }
    return approx;
}

}  // namespace ocs2::mobile_manipulator
