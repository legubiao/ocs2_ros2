#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "ocs2_mobile_manipulator/collision/EnvironmentGeometryInterface.h"

namespace ocs2::mobile_manipulator {

/**
 * @brief Environment Collision Constraint
 *
 * This class implements a state constraint for avoiding collisions between
 * the robot and environment obstacles. The constraint value is:
 *   h = distance - minimumDistance
 * where h > 0 means safe (no collision).
 *
 * The constraint uses coal for distance computation and provides
 * analytical Jacobians based on the nearest points between robot links
 * and obstacles.
 */
class EnvironmentCollisionConstraint : public StateConstraint {
public:
    /**
     * @brief Constructor
     * @param mapping Pinocchio state-input mapping
     * @param envGeomInterface Environment geometry interface (shared)
     * @param minimumDistance Minimum allowed distance between robot and obstacles
     */
    EnvironmentCollisionConstraint(
        const PinocchioStateInputMapping<scalar_t>& mapping,
        std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
        scalar_t minimumDistance);

    ~EnvironmentCollisionConstraint() override = default;

    EnvironmentCollisionConstraint* clone() const override = 0;

    /**
     * @brief Get the number of constraint dimensions
     * @note Returns 0 if no obstacles are present
     */
    size_t getNumConstraints(scalar_t time) const override;

    /**
     * @brief Evaluate the constraint value
     * @param time Current time
     * @param state Current state
     * @param preComputation Pre-computation cache
     * @return Constraint values (h = distance - minimumDistance)
     *
     * @note Requires pinocchio::forwardKinematics() to be called in pre-computation
     */
    vector_t getValue(scalar_t time, const vector_t& state,
                      const PreComputation& preComputation) const override;

    /**
     * @brief Get the linear approximation of the constraint
     * @param time Current time
     * @param state Current state
     * @param preComputation Pre-computation cache
     * @return Linear approximation (f, dfdx)
     *
     * @note Requires pinocchio::forwardKinematics(), updateGlobalPlacements(),
     *       and computeJointJacobians() to be called in pre-computation
     */
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const override;

    /**
     * @brief Get the environment geometry interface
     */
    std::shared_ptr<EnvironmentGeometryInterface> getEnvironmentGeometryInterface() const {
        return envGeomInterface_;
    }

    /**
     * @brief Get the minimum distance setting
     */
    scalar_t getMinimumDistance() const { return minimumDistance_; }

protected:
    /**
     * @brief Get the PinocchioInterface from pre-computation
     * @note To be implemented by derived class for specific robot types
     */
    virtual const PinocchioInterface& getPinocchioInterface(
        const PreComputation& preComputation) const = 0;

    EnvironmentCollisionConstraint(const EnvironmentCollisionConstraint& rhs);

    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface_;
    scalar_t minimumDistance_;
    std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
};

/**
 * @brief Mobile Manipulator specific environment collision constraint
 *
 * This class provides the getPinocchioInterface() implementation for
 * the MobileManipulatorPreComputation class.
 */
class MobileManipulatorEnvironmentCollisionConstraint final
    : public EnvironmentCollisionConstraint {
public:
    MobileManipulatorEnvironmentCollisionConstraint(
        const PinocchioStateInputMapping<scalar_t>& mapping,
        std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
        scalar_t minimumDistance);

    ~MobileManipulatorEnvironmentCollisionConstraint() override = default;

    MobileManipulatorEnvironmentCollisionConstraint* clone() const override;

protected:
    const PinocchioInterface& getPinocchioInterface(
        const PreComputation& preComputation) const override;

private:
    MobileManipulatorEnvironmentCollisionConstraint(
        const MobileManipulatorEnvironmentCollisionConstraint& rhs) = default;
};

} // namespace ocs2::mobile_manipulator
