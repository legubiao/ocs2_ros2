/******************************************************************************
 * Abstract source for HOME joint soft-reference (q_home + runtime enable).
 ******************************************************************************/

#pragma once

#include <ocs2_core/Types.h>

namespace ocs2::mobile_manipulator {

/**
 * Read-only view used by HomeJointReferenceConstraint.
 * Controllers mutate state via the concrete ReferenceManager subclass.
 */
class HomeJointReferenceSource {
public:
    virtual ~HomeJointReferenceSource() = default;

    [[nodiscard]] virtual bool isHomeJointReferenceEnabled() const = 0;
    [[nodiscard]] virtual const vector_t& getHomeJointReference() const = 0;
};

}  // namespace ocs2::mobile_manipulator
