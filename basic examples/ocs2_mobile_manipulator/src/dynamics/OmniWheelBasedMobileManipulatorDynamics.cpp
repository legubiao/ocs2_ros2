/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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
