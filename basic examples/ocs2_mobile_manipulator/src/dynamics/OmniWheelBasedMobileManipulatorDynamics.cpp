#include "ocs2_mobile_manipulator/dynamics/OmniWheelBasedMobileManipulatorDynamics.h"

namespace ocs2::mobile_manipulator
{
    OmniWheelBasedMobileManipulatorDynamics::OmniWheelBasedMobileManipulatorDynamics(
        ManipulatorModelInfo info, const std::string& modelName,
        const std::string& modelFolder /*= "/tmp/ocs2"*/,
        bool recompileLibraries /*= true*/, bool verbose /*= true*/)
        : info_(std::move(info))
    {
        this->initialize(info_.stateDim, info_.inputDim, modelName, modelFolder, recompileLibraries, verbose);
    }


    ad_vector_t OmniWheelBasedMobileManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state,
                                                                       const ad_vector_t& input,
                                                                       const ad_vector_t&) const
    {
        ad_vector_t dxdt(info_.stateDim);
        const auto theta = state(2);
        const auto vx = input(0); // forward velocity in base frame
        const auto vy = input(1); // lateral velocity in base frame
        const auto c = cos(theta);
        const auto s = sin(theta);
        // world-frame planar velocity = R(theta) * [vx; vy], then yaw rate and arm rates
        dxdt << c * vx - s * vy, s * vx + c * vy, input(2), input.tail(info_.armDim);
        return dxdt;
    }
}
