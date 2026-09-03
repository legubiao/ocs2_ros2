/******************************************************************************
 * Smooth Marvin CCS 6/7-axis coupling constraint.
 *
 * The manufacturer's admissible region is an octagon in the (J6, J7) plane.
 * This implementation uses a log-sum-exp smooth maximum of its eight
 * half-spaces. The resulting boundary is globally smooth and conservative:
 * every accepted point also lies inside the manufacturer's polygon.
 ******************************************************************************/

#pragma once

#include <cstddef>
#include <vector>

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2::mobile_manipulator
{
    /**
     * State-only soft inequality constraint keeping (J6, J7) inside a smooth,
     * inward-rounded version of the manufacturer's coupled motion range.
     * Residual >= 0 means that the constraint is satisfied.
     */
    class Joint67CouplingConstraint final : public StateConstraint
    {
    public:
        /** Boundary parameters. Angles and distances are specified in degrees. */
        struct Config
        {
            scalar_t j6LimitDeg{58.0};
            scalar_t j7LimitDeg{78.0};
            scalar_t diagonalLimitDeg{110.5};
            scalar_t diagonalSlope{1.025};
            scalar_t smoothTauDeg{2.0};
            scalar_t safetyMarginDeg{1.0};
        };

        /** Result of the 6/7-axis coupling check for one arm. */
        struct CouplingStatus
        {
            /** Inside the smooth, inward-rounded envelope used by OCS2. */
            bool withinRange{false};
            /** Inside the unsmoothed polygon before safety margin is applied. */
            bool withinRawRange{false};
            scalar_t upperLimitDeg{0.0};
            scalar_t lowerLimitDeg{0.0};
            scalar_t upperMarginDeg{0.0};
            scalar_t lowerMarginDeg{0.0};
            scalar_t joint6MarginDeg{0.0};
            /** Positive inside the actual smooth envelope; negative outside. */
            scalar_t smoothMarginDeg{0.0};
        };

        /** Pure degree-domain check using the same smooth formula as OCS2. */
        static CouplingStatus checkCouplingDeg(scalar_t j6Deg, scalar_t j7Deg,
                                               const Config& config);

        /** State indices for one configured arm. */
        struct ArmCoupling
        {
            size_t j6StateIndex{0};
            size_t j7StateIndex{0};
        };

        Joint67CouplingConstraint(std::vector<ArmCoupling> arms, Config config,
                                  size_t stateDim);

        ~Joint67CouplingConstraint() override = default;

        Joint67CouplingConstraint* clone() const override
        {
            return new Joint67CouplingConstraint(*this);
        }

        size_t getNumConstraints(scalar_t time) const override;

        vector_t getValue(scalar_t time, const vector_t& state,
                          const PreComputation& preComputation) const override;

        VectorFunctionLinearApproximation getLinearApproximation(
            scalar_t time, const vector_t& state,
            const PreComputation& preComputation) const override;

    private:
        struct Evaluation
        {
            scalar_t value{0.0};
            scalar_t derivativeJ6{0.0};
            scalar_t derivativeJ7{0.0};
        };

        Joint67CouplingConstraint(const Joint67CouplingConstraint& other) = default;

        static void validateConfig(const Config& config);
        static Evaluation evaluate(scalar_t q6, scalar_t q7, const Config& config);
        void validateStateSize(const vector_t& state) const;

        std::vector<ArmCoupling> arms_;
        Config configRad_;
        size_t stateDim_{0};
    };
}  // namespace ocs2::mobile_manipulator
