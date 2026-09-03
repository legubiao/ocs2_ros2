#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include "ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h"

namespace ocs2::mobile_manipulator
{
    namespace
    {
        constexpr scalar_t kDegToRad = 3.14159265358979323846 / 180.0;

        Joint67CouplingConstraint makeSingleArmConstraint()
        {
            return Joint67CouplingConstraint({{0, 1}}, {}, 2);
        }

        vector_t stateFromDegrees(scalar_t j6Deg, scalar_t j7Deg)
        {
            vector_t state(2);
            state << j6Deg * kDegToRad, j7Deg * kDegToRad;
            return state;
        }
    }  // namespace

    TEST(Joint67CouplingConstraintTest, SmoothEnvelopeIsConservative)
    {
        const Joint67CouplingConstraint::Config config;

        const auto center = Joint67CouplingConstraint::checkCouplingDeg(0.0, 0.0, config);
        EXPECT_TRUE(center.withinRawRange);
        EXPECT_TRUE(center.withinRange);
        EXPECT_GT(center.smoothMarginDeg, 0.0);

        // This point is inside the original polygon, but intentionally removed
        // by corner smoothing plus the safety margin.
        const auto roundedAway = Joint67CouplingConstraint::checkCouplingDeg(30.0, 77.0, config);
        EXPECT_TRUE(roundedAway.withinRawRange);
        EXPECT_FALSE(roundedAway.withinRange);
        EXPECT_LT(roundedAway.smoothMarginDeg, 0.0);

        const auto smoothInterior = Joint67CouplingConstraint::checkCouplingDeg(30.0, 74.0, config);
        EXPECT_TRUE(smoothInterior.withinRange);

        const auto outside = Joint67CouplingConstraint::checkCouplingDeg(59.0, 0.0, config);
        EXPECT_FALSE(outside.withinRawRange);
        EXPECT_FALSE(outside.withinRange);
    }

    TEST(Joint67CouplingConstraintTest, BoundaryAndGradientAreGloballySmooth)
    {
        auto constraint = makeSingleArmConstraint();
        const PreComputation preComputation;
        EXPECT_EQ(constraint.getOrder(), ConstraintOrder::Linear);

        const std::array<std::array<scalar_t, 2>, 6> samples{{
            {{0.0, 0.0}},
            {{30.0, 77.0}},
            {{-30.0, 77.0}},
            {{30.0, -77.0}},
            {{58.0, 0.0}},
            {{0.0, 78.0}},
        }};
        constexpr scalar_t epsilon = 1e-7;

        for (const auto& sample : samples)
        {
            const vector_t state = stateFromDegrees(sample[0], sample[1]);
            const auto approximation =
                constraint.getLinearApproximation(0.0, state, preComputation);
            EXPECT_TRUE(approximation.f.allFinite());
            EXPECT_TRUE(approximation.dfdx.allFinite());

            for (Eigen::Index column = 0; column < state.size(); ++column)
            {
                vector_t plus = state;
                vector_t minus = state;
                plus(column) += epsilon;
                minus(column) -= epsilon;
                const scalar_t numerical =
                    (constraint.getValue(0.0, plus, preComputation)(0) -
                     constraint.getValue(0.0, minus, preComputation)(0)) /
                    (2.0 * epsilon);
                EXPECT_NEAR(approximation.dfdx(0, column), numerical, 1e-7)
                    << "sample (deg): " << sample[0] << ", " << sample[1];
            }
        }
    }

    TEST(Joint67CouplingConstraintTest, EnvelopeIsSymmetricAndSupportsTwoArms)
    {
        Joint67CouplingConstraint constraint({{5, 6}, {12, 13}}, {}, 14);
        const PreComputation preComputation;
        EXPECT_EQ(constraint.getNumConstraints(0.0), 2U);

        vector_t state = vector_t::Zero(14);
        state(5) = 30.0 * kDegToRad;
        state(6) = 74.0 * kDegToRad;
        state(12) = -30.0 * kDegToRad;
        state(13) = -74.0 * kDegToRad;
        const vector_t value = constraint.getValue(0.0, state, preComputation);
        ASSERT_EQ(value.size(), 2);
        EXPECT_NEAR(value(0), value(1), 1e-12);
        EXPECT_GT(value(0), 0.0);
    }

    TEST(Joint67CouplingConstraintTest, SoftCostUsesFinitePositiveSemidefiniteHessian)
    {
        auto constraint = std::make_unique<Joint67CouplingConstraint>(
            std::vector<Joint67CouplingConstraint::ArmCoupling>{{0, 1}},
            Joint67CouplingConstraint::Config{}, 2);
        auto penalty = std::make_unique<SquaredHingePenalty>(
            SquaredHingePenalty::Config{50.0, 3.0 * kDegToRad});
        StateSoftConstraint cost(std::move(constraint), std::move(penalty));
        const TargetTrajectories targetTrajectories;
        const PreComputation preComputation;

        for (const auto& state : {stateFromDegrees(30.0, 77.0),
                                  stateFromDegrees(60.0, 0.0),
                                  stateFromDegrees(0.0, 80.0)})
        {
            const auto approximation = cost.getQuadraticApproximation(
                0.0, state, targetTrajectories, preComputation);
            ASSERT_TRUE(approximation.dfdxx.allFinite());
            Eigen::SelfAdjointEigenSolver<matrix_t> eigenSolver(approximation.dfdxx);
            ASSERT_EQ(eigenSolver.info(), Eigen::Success);
            EXPECT_GE(eigenSolver.eigenvalues().minCoeff(), -1e-12);
        }
    }

    TEST(Joint67CouplingConstraintTest, RejectsInvalidConfigurationAndState)
    {
        auto invalidConfig = Joint67CouplingConstraint::Config{};
        invalidConfig.smoothTauDeg = 0.0;
        EXPECT_THROW(Joint67CouplingConstraint({{0, 1}}, invalidConfig, 2),
                     std::invalid_argument);
        EXPECT_THROW(Joint67CouplingConstraint({{0, 0}}, {}, 2),
                     std::invalid_argument);

        auto constraint = makeSingleArmConstraint();
        EXPECT_THROW(constraint.getValue(0.0, vector_t::Zero(3), PreComputation{}),
                     std::out_of_range);
    }

}  // namespace ocs2::mobile_manipulator
