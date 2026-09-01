/******************************************************************************
 * Marvin CCS 6/7-axis coupling constraint implementation.
 ******************************************************************************/

#include "ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        constexpr scalar_t kDegToRad = 3.14159265358979323846 / 180.0;

        /** Convert a degree-domain parabola (j7 = a0*j6^2 + a1*j6 + a2) to radians. */
        Joint67CouplingConstraint::Parabola toRadians(const Joint67CouplingConstraint::Parabola& p)
        {
            // q7_rad = (a0 / k) * q6_rad^2 + a1 * q6_rad + (a2 * k), with k = pi/180.
            Joint67CouplingConstraint::Parabola r;
            r.a0 = p.a0 / kDegToRad;
            r.a1 = p.a1;
            r.a2 = p.a2 * kDegToRad;
            return r;
        }

        scalar_t evaluate(const Joint67CouplingConstraint::Parabola& p, scalar_t q6)
        {
            return p.a0 * q6 * q6 + p.a1 * q6 + p.a2;
        }
    }  // namespace

    Joint67CouplingConstraint::CouplingStatus Joint67CouplingConstraint::checkCouplingDeg(
        scalar_t j6Deg, scalar_t j7Deg,
        const Parabola& pp, const Parabola& np,
        const Parabola& nn, const Parabola& pn,
        scalar_t deadbandDeg)
    {
        CouplingStatus status;
        if (std::abs(j6Deg) <= deadbandDeg)
        {
            // Inside the deadband there is no interference limit.
            status.withinRange = true;
            status.upperLimitDeg = std::numeric_limits<scalar_t>::infinity();
            status.lowerLimitDeg = -std::numeric_limits<scalar_t>::infinity();
            status.upperMarginDeg = std::numeric_limits<scalar_t>::infinity();
            status.lowerMarginDeg = std::numeric_limits<scalar_t>::infinity();
            return status;
        }

        const Parabola& upper = (j6Deg >= 0.0) ? pp : np;
        const Parabola& lower = (j6Deg >= 0.0) ? pn : nn;
        status.upperLimitDeg = evaluate(upper, j6Deg);
        status.lowerLimitDeg = evaluate(lower, j6Deg);
        status.upperMarginDeg = status.upperLimitDeg - j7Deg;
        status.lowerMarginDeg = j7Deg - status.lowerLimitDeg;
        status.withinRange = (status.upperMarginDeg >= 0.0) && (status.lowerMarginDeg >= 0.0);
        return status;
    }

    Joint67CouplingConstraint::Joint67CouplingConstraint(std::vector<ArmCoupling> arms, size_t stateDim)
        : StateConstraint(ConstraintOrder::Quadratic)
        , stateDim_(stateDim)
    {
        arms_.reserve(arms.size());
        for (auto& arm : arms)
        {
            if (arm.j6StateIndex >= stateDim || arm.j7StateIndex >= stateDim)
            {
                throw std::invalid_argument(
                    "[Joint67CouplingConstraint] J6/J7 state index is outside state dimension");
            }
            arm.pp = toRadians(arm.pp);
            arm.np = toRadians(arm.np);
            arm.nn = toRadians(arm.nn);
            arm.pn = toRadians(arm.pn);
            arm.deadbandDeg = arm.deadbandDeg * kDegToRad;  // store as radians
            arms_.push_back(arm);
        }
    }

    void Joint67CouplingConstraint::validateStateSize(const vector_t& state) const
    {
        if (static_cast<size_t>(state.size()) != stateDim_)
        {
            throw std::out_of_range("[Joint67CouplingConstraint] state dimension mismatch");
        }
    }

    size_t Joint67CouplingConstraint::getNumConstraints(scalar_t /*time*/) const
    {
        return arms_.size() * 2;
    }

    vector_t Joint67CouplingConstraint::getValue(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& /*preComputation*/) const
    {
        validateStateSize(state);

        vector_t value(static_cast<Eigen::Index>(arms_.size() * 2));
        Eigen::Index row = 0;
        for (const auto& arm : arms_)
        {
            const scalar_t q6 = state(static_cast<Eigen::Index>(arm.j6StateIndex));
            const scalar_t q7 = state(static_cast<Eigen::Index>(arm.j7StateIndex));

            if (std::abs(q6) <= arm.deadbandDeg)
            {
                // Within the deadband the robot applies no interference limit.
                // A constant positive residual => penalty zero, gradient zero.
                value(row++) = 1.0;
                value(row++) = 1.0;
                continue;
            }

            const Parabola& upper = (q6 >= 0.0) ? arm.pp : arm.np;
            const Parabola& lower = (q6 >= 0.0) ? arm.pn : arm.nn;
            value(row++) = evaluate(upper, q6) - q7;  // >= 0 inside
            value(row++) = q7 - evaluate(lower, q6);  // >= 0 inside
        }
        return value;
    }

    VectorFunctionLinearApproximation Joint67CouplingConstraint::getLinearApproximation(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& /*preComputation*/) const
    {
        validateStateSize(state);

        const Eigen::Index nv = static_cast<Eigen::Index>(arms_.size() * 2);
        VectorFunctionLinearApproximation approx;
        approx.setZero(nv, state.size(), 0);

        Eigen::Index row = 0;
        for (const auto& arm : arms_)
        {
            const Eigen::Index j6 = static_cast<Eigen::Index>(arm.j6StateIndex);
            const Eigen::Index j7 = static_cast<Eigen::Index>(arm.j7StateIndex);
            const scalar_t q6 = state(j6);

            if (std::abs(q6) <= arm.deadbandDeg)
            {
                approx.f(row) = 1.0;
                approx.f(row + 1) = 1.0;
                row += 2;
                continue;
            }

            const Parabola& upper = (q6 >= 0.0) ? arm.pp : arm.np;
            const Parabola& lower = (q6 >= 0.0) ? arm.pn : arm.nn;

            const scalar_t dUpper = 2.0 * upper.a0 * q6 + upper.a1;
            const scalar_t dLower = 2.0 * lower.a0 * q6 + lower.a1;

            // h_upper = upper(q6) - q7
            approx.f(row) = evaluate(upper, q6) - state(j7);
            approx.dfdx(row, j6) = dUpper;
            approx.dfdx(row, j7) = -1.0;

            // h_lower = q7 - lower(q6)
            approx.f(row + 1) = state(j7) - evaluate(lower, q6);
            approx.dfdx(row + 1, j6) = -dLower;
            approx.dfdx(row + 1, j7) = 1.0;

            row += 2;
        }
        return approx;
    }

    VectorFunctionQuadraticApproximation Joint67CouplingConstraint::getQuadraticApproximation(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& /*preComputation*/) const
    {
        validateStateSize(state);

        const Eigen::Index nv = static_cast<Eigen::Index>(arms_.size() * 2);
        VectorFunctionQuadraticApproximation approx;
        approx.setZero(nv, state.size(), 0);

        Eigen::Index row = 0;
        for (const auto& arm : arms_)
        {
            const Eigen::Index j6 = static_cast<Eigen::Index>(arm.j6StateIndex);
            const Eigen::Index j7 = static_cast<Eigen::Index>(arm.j7StateIndex);
            const scalar_t q6 = state(j6);

            if (std::abs(q6) <= arm.deadbandDeg)
            {
                approx.f(row) = 1.0;
                approx.f(row + 1) = 1.0;
                row += 2;
                continue;
            }

            const Parabola& upper = (q6 >= 0.0) ? arm.pp : arm.np;
            const Parabola& lower = (q6 >= 0.0) ? arm.pn : arm.nn;

            const scalar_t dUpper = 2.0 * upper.a0 * q6 + upper.a1;
            const scalar_t dLower = 2.0 * lower.a0 * q6 + lower.a1;

            // h_upper = upper(q6) - q7
            approx.f(row) = evaluate(upper, q6) - state(j7);
            approx.dfdx(row, j6) = dUpper;
            approx.dfdx(row, j7) = -1.0;
            approx.dfdxx[static_cast<size_t>(row)](j6, j6) = 2.0 * upper.a0;

            // h_lower = q7 - lower(q6)
            approx.f(row + 1) = state(j7) - evaluate(lower, q6);
            approx.dfdx(row + 1, j6) = -dLower;
            approx.dfdx(row + 1, j7) = 1.0;
            approx.dfdxx[static_cast<size_t>(row + 1)](j6, j6) = -2.0 * lower.a0;

            row += 2;
        }
        return approx;
    }

}  // namespace ocs2::mobile_manipulator
