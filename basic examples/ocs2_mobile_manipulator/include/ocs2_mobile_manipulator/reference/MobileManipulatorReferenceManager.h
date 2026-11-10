/******************************************************************************
 * ReferenceManager with switchable HOME joint soft-reference state.
 ******************************************************************************/

#pragma once

#include <atomic>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_mobile_manipulator/reference/HomeJointReferenceSource.h"

namespace ocs2::mobile_manipulator {

/**
 * Same TargetTrajectories API as ReferenceManager, plus arm-local q_home
 * used by HomeJointReferenceConstraint.
 */
class MobileManipulatorReferenceManager : public ReferenceManager,
                                          public HomeJointReferenceSource {
public:
    MobileManipulatorReferenceManager() = default;

    void setHomeJointReferenceEnabled(bool enabled) {
        homeJointReferenceEnabled_.store(enabled, std::memory_order_relaxed);
    }

    [[nodiscard]] bool isHomeJointReferenceEnabled() const override {
        return homeJointReferenceEnabled_.load(std::memory_order_relaxed);
    }

    void setHomeJointReference(const vector_t& armJoints) { homeJointReference_ = armJoints; }

    [[nodiscard]] const vector_t& getHomeJointReference() const override {
        return homeJointReference_;
    }

private:
    std::atomic<bool> homeJointReferenceEnabled_{false};
    vector_t homeJointReference_;
};

}  // namespace ocs2::mobile_manipulator
