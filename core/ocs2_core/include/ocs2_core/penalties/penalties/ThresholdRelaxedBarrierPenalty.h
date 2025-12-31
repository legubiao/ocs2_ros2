#pragma once

#include <ocs2_core/penalties/penalties/PenaltyBase.h>

namespace ocs2
{
    /**
     * Implements a threshold-based relaxed barrier function for a single inequality constraint h >= 0
     *
     * The penalty function is only active when h < activationThreshold:
     *
     *   p(h) = 0                                                      if h >= activationThreshold
     *   p(h) = -mu * ln(h)                                            if delta < h < activationThreshold
     *   p(h) = -mu * ln(delta) + mu * 0.5 * ((h-2*delta)/delta)^2 - 0.5)  if h <= delta
     *
     * where mu >= 0, delta >= 0, and activationThreshold > 0 are user defined parameters.
     *
     * The function ensures C1 continuity at h = activationThreshold by subtracting a baseline penalty
     * so that p(activationThreshold) = 0.
     */
    class ThresholdRelaxedBarrierPenalty final : public PenaltyBase
    {
    public:
        /**
         * Configuration object for the threshold relaxed barrier penalty.
         * mu : scaling factor
         * delta: relaxation parameter for numerical stability near h=0
         * activationThreshold: penalty is only applied when h < activationThreshold
         */
        struct Config
        {
            Config() : Config(1e-2, 1e-3, 0.05)
            {
            }

            Config(scalar_t muParam, scalar_t deltaParam, scalar_t activationThresholdParam)
                : mu(muParam), delta(deltaParam), activationThreshold(activationThresholdParam)
            {
            }

            scalar_t mu;
            scalar_t delta;
            scalar_t activationThreshold;
        };

        /**
         * Constructor
         * @param [in] config: Configuration object containing mu, delta, and activationThreshold.
         */
        explicit ThresholdRelaxedBarrierPenalty(Config config);

        ~ThresholdRelaxedBarrierPenalty() override = default;
        ThresholdRelaxedBarrierPenalty* clone() const override { return new ThresholdRelaxedBarrierPenalty(*this); }
        std::string name() const override { return "ThresholdRelaxedBarrierPenalty"; }

        scalar_t getValue(scalar_t t, scalar_t h) const override;
        scalar_t getDerivative(scalar_t t, scalar_t h) const override;
        scalar_t getSecondDerivative(scalar_t t, scalar_t h) const override;

    private:
        ThresholdRelaxedBarrierPenalty(const ThresholdRelaxedBarrierPenalty& other) = default;

        /** Compute the raw relaxed barrier penalty value (without threshold offset) */
        scalar_t getRawValue(scalar_t h) const;

        /** Compute the raw relaxed barrier penalty derivative */
        scalar_t getRawDerivative(scalar_t h) const;

        /** Compute the raw relaxed barrier penalty second derivative */
        scalar_t getRawSecondDerivative(scalar_t h) const;

        const Config config_;
        const scalar_t baselinePenalty_;
        // penalty value at activationThreshold, used to ensure p(activationThreshold) = 0
    };
} // namespace ocs2
