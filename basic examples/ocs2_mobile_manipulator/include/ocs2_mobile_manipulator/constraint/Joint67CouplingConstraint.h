/******************************************************************************
 * Marvin CCS 6/7-axis coupling constraint.
 *
 * The CCS (compact coupled structure) wrist of the Marvin M3S/M6S/M20S arms has
 * a coupled motion range between joint 6 (wrist pitch) and joint 7 (wrist
 * roll). The allowed region is bounded by the quadratic curves stored in the
 * robot's MvKDCfg "BD" field (four quadrants: ++ / -+ / -- / +-):
 *
 *     j7 = a0 * j6^2 + a1 * j6 + a2      (angles in degrees)
 *
 * This mirrors the interference check performed by the robot's own IK
 * (see TJ_FX_ROBOT_CONTRL_SDK kinematicsSDK/FxRobot.cpp, lmtj67_pp/np/nn/pn).
 ******************************************************************************/

#pragma once

#include <cstddef>
#include <vector>

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2::mobile_manipulator
{
    /**
     * State-only soft inequality constraint keeping (J6, J7) inside the coupled
     * motion range. Residual convention follows ocs2::RelaxedBarrierPenalty:
     * residual >= 0 means the constraint is satisfied.
     */
    class Joint67CouplingConstraint final : public StateConstraint
    {
    public:
        /** Quadratic polynomial coefficients (input in degrees): j7 = a0*j6^2 + a1*j6 + a2. */
        struct Parabola
        {
            scalar_t a0{0.0};
            scalar_t a1{0.0};
            scalar_t a2{0.0};
        };

        /** Result of the 6/7-axis coupling check for a single (J6, J7) pair. */
        struct CouplingStatus
        {
            bool withinRange{false};
            scalar_t upperLimitDeg{0.0};  // J7 upper bound at the given J6
            scalar_t lowerLimitDeg{0.0};  // J7 lower bound at the given J6
            scalar_t upperMarginDeg{0.0}; // positive: J7 below upper bound; negative: violation
            scalar_t lowerMarginDeg{0.0}; // positive: J7 above lower bound; negative: violation
        };

        /**
         * Pure 6/7-axis coupling check (degrees). Same formula as the robot's IK
         * interference test: j7 = a0*j6^2 + a1*j6 + a2 per quadrant.
         *
         * @param j6Deg       6th joint angle [deg].
         * @param j7Deg       7th joint angle [deg].
         * @param pp          ++ quadrant parabola (J6>=0, J7 upper bound).
         * @param np          -+ quadrant parabola (J6<0,  J7 upper bound).
         * @param nn          -- quadrant parabola (J6<0,  J7 lower bound).
         * @param pn          +- quadrant parabola (J6>=0, J7 lower bound).
         * @param deadbandDeg |J6| below this is not interference-limited (SDK uses 1.0).
         */
        static CouplingStatus checkCouplingDeg(scalar_t j6Deg, scalar_t j7Deg,
                                               const Parabola& pp, const Parabola& np,
                                               const Parabola& nn, const Parabola& pn,
                                               scalar_t deadbandDeg = 1.0);

        /** Coupling parameters for one arm. */
        struct ArmCoupling
        {
            size_t j6StateIndex{0};   // absolute state index of the J6 joint angle
            size_t j7StateIndex{0};   // absolute state index of the J7 joint angle
            Parabola pp;              // J6 >= 0, J7 upper bound (quadrant ++)
            Parabola np;              // J6 <  0, J7 upper bound (quadrant -+)
            Parabola nn;              // J6 <  0, J7 lower bound (quadrant --)
            Parabola pn;              // J6 >= 0, J7 lower bound (quadrant +-)
            scalar_t deadbandDeg{1.0};// |J6| below this (deg) has no interference limit
        };

        /**
         * @param arms      Coupling parameters for each configured arm (left/right).
         * @param stateDim  Total state dimension (used for validation).
         */
        Joint67CouplingConstraint(std::vector<ArmCoupling> arms, size_t stateDim);

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

        VectorFunctionQuadraticApproximation getQuadraticApproximation(
            scalar_t time, const vector_t& state,
            const PreComputation& preComputation) const override;

    private:
        Joint67CouplingConstraint(const Joint67CouplingConstraint& other) = default;

        void validateStateSize(const vector_t& state) const;

        std::vector<ArmCoupling> arms_;
        size_t stateDim_{0};
    };
}  // namespace ocs2::mobile_manipulator
