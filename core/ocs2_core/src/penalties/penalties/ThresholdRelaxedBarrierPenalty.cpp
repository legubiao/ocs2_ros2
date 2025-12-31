#include <cmath>
#include <ocs2_core/penalties/penalties/ThresholdRelaxedBarrierPenalty.h>

namespace ocs2
{
    ThresholdRelaxedBarrierPenalty::ThresholdRelaxedBarrierPenalty(Config config)
        : config_(std::move(config)),
          baselinePenalty_(getRawValue(config_.activationThreshold))
    {
    }

    scalar_t ThresholdRelaxedBarrierPenalty::getRawValue(scalar_t h) const
    {
        if (h > config_.delta)
        {
            return -config_.mu * std::log(h);
        }
        const scalar_t delta_h = (h - 2.0 * config_.delta) / config_.delta;
        return config_.mu * (-std::log(config_.delta) + 0.5 * delta_h * delta_h - 0.5);
    }

    scalar_t ThresholdRelaxedBarrierPenalty::getRawDerivative(scalar_t h) const
    {
        if (h > config_.delta)
        {
            return -config_.mu / h;
        }
        return config_.mu * ((h - 2.0 * config_.delta) / (config_.delta * config_.delta));
    }

    scalar_t ThresholdRelaxedBarrierPenalty::getRawSecondDerivative(scalar_t h) const
    {
        if (h > config_.delta)
        {
            return config_.mu / (h * h);
        }
        return config_.mu / (config_.delta * config_.delta);
    }

    scalar_t ThresholdRelaxedBarrierPenalty::getValue(scalar_t t, scalar_t h) const
    {
        // No penalty when h >= activationThreshold
        if (h >= config_.activationThreshold)
        {
            return 0.0;
        }
        // Subtract baseline to ensure continuity: p(activationThreshold) = 0
        return getRawValue(h) - baselinePenalty_;
    }

    scalar_t ThresholdRelaxedBarrierPenalty::getDerivative(scalar_t t, scalar_t h) const
    {
        // Zero derivative when h >= activationThreshold
        if (h >= config_.activationThreshold)
        {
            return 0.0;
        }
        return getRawDerivative(h);
    }

    scalar_t ThresholdRelaxedBarrierPenalty::getSecondDerivative(scalar_t t, scalar_t h) const
    {
        // Zero second derivative when h >= activationThreshold
        if (h >= config_.activationThreshold)
        {
            return 0.0;
        }
        return getRawSecondDerivative(h);
    }
} // namespace ocs2
