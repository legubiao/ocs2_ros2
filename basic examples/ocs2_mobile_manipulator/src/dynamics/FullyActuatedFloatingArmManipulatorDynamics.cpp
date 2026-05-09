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

#include "ocs2_mobile_manipulator/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"

#include <cppad/cppad.hpp>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        inline ocs2::ad_scalar_t smoothOutwardScale(const ocs2::ad_scalar_t& distToBound, ocs2::scalar_t eps)
        {
            const ocs2::ad_scalar_t x = distToBound / ocs2::ad_scalar_t(eps);
            return ocs2::ad_scalar_t(0.5) * (CppAD::tanh(x) + ocs2::ad_scalar_t(1.0));
        }
    }

    FullyActuatedFloatingArmManipulatorDynamics::FullyActuatedFloatingArmManipulatorDynamics(
        const ManipulatorModelInfo& info,
        const std::string& modelName,
        vector_t positionLowerLimit,
        vector_t positionUpperLimit,
        scalar_t jointLimitEps,
        const std::string& modelFolder /*= "/tmp/ocs2"*/,
        bool recompileLibraries /*= true*/,
        bool verbose /*= true*/)
        : positionLowerLimit_(std::move(positionLowerLimit)),
          positionUpperLimit_(std::move(positionUpperLimit)),
          jointLimitEps_(jointLimitEps)
    {
        this->initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
    }


    ad_vector_t FullyActuatedFloatingArmManipulatorDynamics::systemFlowMap(
        ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
        const ad_vector_t&) const
    {
        ad_vector_t dxdt = input;

        // Bake in joint position limits (arm joints only; they are at the end of the state vector).
        const int armDim = static_cast<int>(positionLowerLimit_.size());
        if (jointLimitEps_ > 0.0 && armDim > 0 && positionUpperLimit_.size() == armDim &&
            armDim <= dxdt.size() && armDim <= state.size())
        {
            const int armStart = static_cast<int>(state.size() - armDim);
            for (int j = 0; j < armDim; ++j)
            {
                const scalar_t lo = positionLowerLimit_(j);
                const scalar_t hi = positionUpperLimit_(j);
                if (!(hi > lo))
                {
                    continue;
                }
                const int idx = armStart + j;
                const auto vj = dxdt(idx);
                const auto sPos = smoothOutwardScale(ocs2::ad_scalar_t(hi) - state(idx), jointLimitEps_);
                const auto sNeg = smoothOutwardScale(state(idx) - ocs2::ad_scalar_t(lo), jointLimitEps_);
                dxdt(idx) = CppAD::CondExpGt(vj, ocs2::ad_scalar_t(0.0), vj * sPos,
                                            CppAD::CondExpLt(vj, ocs2::ad_scalar_t(0.0), vj * sNeg, vj));
            }
        }

        return dxdt;
    }
}
