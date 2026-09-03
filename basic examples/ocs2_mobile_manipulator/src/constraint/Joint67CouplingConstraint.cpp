/******************************************************************************
 * Smooth Marvin CCS 6/7-axis coupling constraint implementation.
 ******************************************************************************/

#include "ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        constexpr scalar_t kDegToRad = 3.14159265358979323846 / 180.0;

        Joint67CouplingConstraint::Config convertAnglesToRadians(
            Joint67CouplingConstraint::Config config)
        {
            config.j6LimitDeg *= kDegToRad;
            config.j7LimitDeg *= kDegToRad;
            config.diagonalLimitDeg *= kDegToRad;
            config.smoothTauDeg *= kDegToRad;
            config.safetyMarginDeg *= kDegToRad;
            return config;
        }

        bool isFinite(scalar_t value)
        {
            return std::isfinite(value);
        }
    }  // namespace

    void Joint67CouplingConstraint::validateConfig(const Config& config)
    {
        if (!isFinite(config.j6LimitDeg) || config.j6LimitDeg <= 0.0 ||
            !isFinite(config.j7LimitDeg) || config.j7LimitDeg <= 0.0 ||
            !isFinite(config.diagonalLimitDeg) || config.diagonalLimitDeg <= 0.0 ||
            !isFinite(config.diagonalSlope) || config.diagonalSlope <= 0.0 ||
            !isFinite(config.smoothTauDeg) || config.smoothTauDeg <= 0.0 ||
            !isFinite(config.safetyMarginDeg) || config.safetyMarginDeg < 0.0)
        {
            throw std::invalid_argument(
                "[Joint67CouplingConstraint] invalid boundary configuration");
        }
    }

    Joint67CouplingConstraint::Evaluation Joint67CouplingConstraint::evaluate(
        scalar_t q6, scalar_t q7, const Config& config)
    {
        const scalar_t slope = config.diagonalSlope;
        const scalar_t diagonalNormal = std::sqrt(slope * slope + 1.0);

        // Each g_i is the signed Euclidean distance to one polygon half-space:
        // g_i <= 0 is inside. SmoothMax(g_i) >= max(g_i), therefore requiring
        // -SmoothMax(g_i)-margin >= 0 is a conservative rounded envelope.
        const std::array<scalar_t, 8> g{
            q6 - config.j6LimitDeg,
            -q6 - config.j6LimitDeg,
            q7 - config.j7LimitDeg,
            -q7 - config.j7LimitDeg,
            (slope * q6 + q7 - config.diagonalLimitDeg) / diagonalNormal,
            (slope * q6 - q7 - config.diagonalLimitDeg) / diagonalNormal,
            (-slope * q6 + q7 - config.diagonalLimitDeg) / diagonalNormal,
            (-slope * q6 - q7 - config.diagonalLimitDeg) / diagonalNormal,
        };
        const std::array<scalar_t, 8> dgJ6{
            1.0, -1.0, 0.0, 0.0,
            slope / diagonalNormal, slope / diagonalNormal,
            -slope / diagonalNormal, -slope / diagonalNormal,
        };
        const std::array<scalar_t, 8> dgJ7{
            0.0, 0.0, 1.0, -1.0,
            1.0 / diagonalNormal, -1.0 / diagonalNormal,
            1.0 / diagonalNormal, -1.0 / diagonalNormal,
        };

        const scalar_t gMax = *std::max_element(g.begin(), g.end());
        scalar_t exponentialSum = 0.0;
        std::array<scalar_t, 8> exponentials{};
        for (size_t i = 0; i < g.size(); ++i)
        {
            exponentials[i] = std::exp((g[i] - gMax) / config.smoothTauDeg);
            exponentialSum += exponentials[i];
        }

        const scalar_t smoothMax =
            gMax + config.smoothTauDeg * std::log(exponentialSum);
        Evaluation result;
        result.value = -smoothMax - config.safetyMarginDeg;
        for (size_t i = 0; i < g.size(); ++i)
        {
            const scalar_t weight = exponentials[i] / exponentialSum;
            result.derivativeJ6 -= weight * dgJ6[i];
            result.derivativeJ7 -= weight * dgJ7[i];
        }
        return result;
    }

    Joint67CouplingConstraint::CouplingStatus Joint67CouplingConstraint::checkCouplingDeg(
        scalar_t j6Deg, scalar_t j7Deg, const Config& config)
    {
        validateConfig(config);

        CouplingStatus status;
        status.joint6MarginDeg = config.j6LimitDeg - std::abs(j6Deg);
        const scalar_t j7HalfRange = std::min(
            config.j7LimitDeg,
            config.diagonalLimitDeg - config.diagonalSlope * std::abs(j6Deg));
        status.upperLimitDeg = j7HalfRange;
        status.lowerLimitDeg = -j7HalfRange;
        status.upperMarginDeg = status.upperLimitDeg - j7Deg;
        status.lowerMarginDeg = j7Deg - status.lowerLimitDeg;
        status.withinRawRange = status.joint6MarginDeg >= 0.0 &&
                                status.upperMarginDeg >= 0.0 &&
                                status.lowerMarginDeg >= 0.0;
        status.smoothMarginDeg = evaluate(j6Deg, j7Deg, config).value;
        status.withinRange = status.smoothMarginDeg >= 0.0;
        return status;
    }

    Joint67CouplingConstraint::Joint67CouplingConstraint(
        std::vector<ArmCoupling> arms, Config config, size_t stateDim)
        : StateConstraint(ConstraintOrder::Linear)
        , arms_(std::move(arms))
        , configRad_(convertAnglesToRadians(config))
        , stateDim_(stateDim)
    {
        validateConfig(config);
        if (arms_.empty())
        {
            throw std::invalid_argument(
                "[Joint67CouplingConstraint] at least one arm is required");
        }
        for (const auto& arm : arms_)
        {
            if (arm.j6StateIndex >= stateDim || arm.j7StateIndex >= stateDim ||
                arm.j6StateIndex == arm.j7StateIndex)
            {
                throw std::invalid_argument(
                    "[Joint67CouplingConstraint] invalid J6/J7 state index");
            }
        }
        if (evaluate(0.0, 0.0, configRad_).value <= 0.0)
        {
            throw std::invalid_argument(
                "[Joint67CouplingConstraint] smoothing and margin leave no feasible center");
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
        return arms_.size();
    }

    vector_t Joint67CouplingConstraint::getValue(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& /*preComputation*/) const
    {
        validateStateSize(state);

        vector_t value(static_cast<Eigen::Index>(arms_.size()));
        for (size_t i = 0; i < arms_.size(); ++i)
        {
            const auto& arm = arms_[i];
            value(static_cast<Eigen::Index>(i)) = evaluate(
                state(static_cast<Eigen::Index>(arm.j6StateIndex)),
                state(static_cast<Eigen::Index>(arm.j7StateIndex)), configRad_).value;
        }
        return value;
    }

    VectorFunctionLinearApproximation Joint67CouplingConstraint::getLinearApproximation(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& /*preComputation*/) const
    {
        validateStateSize(state);

        const Eigen::Index constraintCount = static_cast<Eigen::Index>(arms_.size());
        VectorFunctionLinearApproximation approximation;
        approximation.setZero(constraintCount, state.size(), 0);

        for (size_t i = 0; i < arms_.size(); ++i)
        {
            const auto& arm = arms_[i];
            const Eigen::Index row = static_cast<Eigen::Index>(i);
            const Eigen::Index j6 = static_cast<Eigen::Index>(arm.j6StateIndex);
            const Eigen::Index j7 = static_cast<Eigen::Index>(arm.j7StateIndex);
            const Evaluation result = evaluate(state(j6), state(j7), configRad_);
            approximation.f(row) = result.value;
            approximation.dfdx(row, j6) = result.derivativeJ6;
            approximation.dfdx(row, j7) = result.derivativeJ7;
        }
        return approximation;
    }

}  // namespace ocs2::mobile_manipulator
