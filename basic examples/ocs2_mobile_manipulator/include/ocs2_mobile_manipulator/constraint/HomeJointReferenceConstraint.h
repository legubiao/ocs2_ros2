/******************************************************************************
 * Soft joint-space reference toward controller HOME / home_joint_reference.
 *
 * Active only when HomeJointReferenceSource::isHomeJointReferenceEnabled().
 * Residual is (q_i - q_home_i) for each tracked arm-local joint.
 ******************************************************************************/

#pragma once

#include <cstddef>
#include <vector>

#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_mobile_manipulator/reference/HomeJointReferenceSource.h"

namespace ocs2::mobile_manipulator {

class HomeJointReferenceConstraint final : public StateConstraint {
public:
    HomeJointReferenceConstraint(const HomeJointReferenceSource& referenceSource,
                                 size_t baseStateDim,
                                 std::vector<size_t> trackedArmIndices);

    ~HomeJointReferenceConstraint() override = default;

    HomeJointReferenceConstraint* clone() const override {
        return new HomeJointReferenceConstraint(*this);
    }

    size_t getNumConstraints(scalar_t time) const override;

    vector_t getValue(scalar_t time, const vector_t& state,
                      const PreComputation& preComputation) const override;

    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const override;

    size_t trackedJointCount() const { return trackedArmIndices_.size(); }

private:
    HomeJointReferenceConstraint(const HomeJointReferenceConstraint& other) = default;

    bool isActive(scalar_t time) const;
    void validateStateSize(const vector_t& state) const;

    const HomeJointReferenceSource* referenceSourcePtr_{nullptr};
    size_t baseStateDim_{0};
    std::vector<size_t> trackedArmIndices_;
};

}  // namespace ocs2::mobile_manipulator
