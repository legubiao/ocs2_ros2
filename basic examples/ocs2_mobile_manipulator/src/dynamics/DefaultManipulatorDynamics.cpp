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

#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"

#include <cppad/cppad.hpp>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        inline ocs2::ad_scalar_t smoothOutwardScale(const ocs2::ad_scalar_t& distToBound, ocs2::scalar_t eps)
        {
            // eps > 0, scale in (0,1): ~1 when dist>>eps, ~0 when dist<<-eps
            const ocs2::ad_scalar_t x = distToBound / ocs2::ad_scalar_t(eps);
            return ocs2::ad_scalar_t(0.5) * (CppAD::tanh(x) + ocs2::ad_scalar_t(1.0));
        }
    }

    DefaultManipulatorDynamics::DefaultManipulatorDynamics(const ManipulatorModelInfo& info,
                                                           const std::string& modelName,
                                                           vector_t positionLowerLimit,
                                                           vector_t positionUpperLimit,
                                                           scalar_t jointLimitEps,
                                                           const std::string& modelFolder,
                                                           bool recompileLibraries /*= true*/,
                                                           bool verbose /*= true*/)
        : positionLowerLimit_(std::move(positionLowerLimit)),
          positionUpperLimit_(std::move(positionUpperLimit)),
          jointLimitEps_(jointLimitEps)
    {
        this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
    }


    ad_vector_t DefaultManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state,
                                                          const ad_vector_t& input,
                                                          const ad_vector_t&) const
    {
        ad_vector_t dxdt = input;

        // Bake in joint position limits as a smooth outward velocity attenuation.
        // This improves model match when the real plant saturates at joint limits.
        if (jointLimitEps_ > 0.0 && positionLowerLimit_.size() == dxdt.size() && positionUpperLimit_.size() == dxdt.size())
        {
            for (int i = 0; i < dxdt.size(); ++i)
            {
                const scalar_t lo = positionLowerLimit_(i);
                const scalar_t hi = positionUpperLimit_(i);
                if (!(hi > lo))
                {
                    continue;
                }
                const auto v = dxdt(i);
                // only attenuate outward motion
                const auto sPos = smoothOutwardScale(ocs2::ad_scalar_t(hi) - state(i), jointLimitEps_);
                const auto sNeg = smoothOutwardScale(state(i) - ocs2::ad_scalar_t(lo), jointLimitEps_);
                dxdt(i) = CppAD::CondExpGt(v, ocs2::ad_scalar_t(0.0), v * sPos,
                                          CppAD::CondExpLt(v, ocs2::ad_scalar_t(0.0), v * sNeg, v));
            }
        }
        return dxdt;
    }
} // namespace ocs2::mobile_manipulator
