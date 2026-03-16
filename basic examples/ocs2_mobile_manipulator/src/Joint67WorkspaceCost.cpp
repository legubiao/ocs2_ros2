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

#include "ocs2_mobile_manipulator/Joint67WorkspaceCost.h"
#include <iostream>
#include <iomanip>

namespace ocs2::mobile_manipulator
{
    Joint67WorkspaceCost::Joint67WorkspaceCost(size_t j6Index, size_t j7Index, scalar_t insideCost, scalar_t outsideCost, bool enableLogging)
        : j6Index_(j6Index), j7Index_(j7Index), insideCost_(insideCost), outsideCost_(outsideCost), enableLogging_(enableLogging), logCounter_(0)
    {
    }

    Joint67WorkspaceCost* Joint67WorkspaceCost::clone() const
    {
        return new Joint67WorkspaceCost(*this);
    }

    std::vector<std::pair<scalar_t, scalar_t>> Joint67WorkspaceCost::getOctagonVertices()
    {
        return {
            {-20.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD},   // (-0.349, 1.571)
            {20.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD},    // (0.349, 1.571)
            {60.0 * DEG_TO_RAD, 49.0 * DEG_TO_RAD},    // (1.047, 0.855)
            {60.0 * DEG_TO_RAD, -49.0 * DEG_TO_RAD},   // (1.047, -0.855)
            {20.0 * DEG_TO_RAD, -90.0 * DEG_TO_RAD},   // (0.349, -1.571)
            {-20.0 * DEG_TO_RAD, -90.0 * DEG_TO_RAD},  // (-0.349, -1.571)
            {-60.0 * DEG_TO_RAD, -49.0 * DEG_TO_RAD},  // (-1.047, -0.855)
            {-60.0 * DEG_TO_RAD, 49.0 * DEG_TO_RAD}    // (-1.047, 0.855)
        };
    }

    bool Joint67WorkspaceCost::isInsideWorkspace(scalar_t j6, scalar_t j7) const
    {
        // Use ray casting algorithm to check if point is inside polygon
        const auto vertices = getOctagonVertices();
        bool inside = false;
        
        for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
        {
            const auto& vi = vertices[i];
            const auto& vj = vertices[j];
            
            // Check if ray from point crosses edge
            if (((vi.second > j7) != (vj.second > j7)) &&
                (j6 < (vj.first - vi.first) * (j7 - vi.second) / (vj.second - vi.second) + vi.first))
            {
                inside = !inside;
            }
        }
        
        return inside;
    }


    scalar_t Joint67WorkspaceCost::getValue(scalar_t time, const vector_t& state,
                                           const TargetTrajectories& targetTrajectories,
                                           const PreComputation& /* preComp */) const
    {
        if (j6Index_ >= state.size() || j7Index_ >= state.size())
        {
            return 0.0;
        }
        
        scalar_t j6_current = state[j6Index_];
        scalar_t j7_current = state[j7Index_];
        
        bool inside = isInsideWorkspace(j6_current, j7_current);
        scalar_t cost = inside ? insideCost_ : outsideCost_;
        
        // Log only when outside the workspace boundary
        if (enableLogging_ && !inside)
        {
            logCounter_++;
            if (logCounter_ % LOG_INTERVAL == 0)
            {
                // Convert radians to degrees for readability
                constexpr scalar_t RAD_TO_DEG = 180.0 / M_PI;
                scalar_t j6_current_deg = j6_current * RAD_TO_DEG;
                scalar_t j7_current_deg = j7_current * RAD_TO_DEG;
                
                // Try to get planned/desired state from targetTrajectories
                scalar_t j6_planned_deg = 0.0;
                scalar_t j7_planned_deg = 0.0;
                bool has_planned = false;
                
                if (!targetTrajectories.empty() && j6Index_ < targetTrajectories.getDesiredState(time).size() && 
                    j7Index_ < targetTrajectories.getDesiredState(time).size())
                {
                    try {
                        vector_t desired_state = targetTrajectories.getDesiredState(time);
                        if (j6Index_ < desired_state.size() && j7Index_ < desired_state.size())
                        {
                            scalar_t j6_planned = desired_state[j6Index_];
                            scalar_t j7_planned = desired_state[j7Index_];
                            j6_planned_deg = j6_planned * RAD_TO_DEG;
                            j7_planned_deg = j7_planned * RAD_TO_DEG;
                            has_planned = true;
                        }
                    } catch (...) {
                        // If getDesiredState fails, just skip planned values
                        has_planned = false;
                    }
                }
                
                std::cerr << std::fixed << std::setprecision(3)
                          << "[Joint67WorkspaceCost] WARNING: Outside workspace boundary!"
                          << " t=" << std::setw(8) << time
                          << " | J6_current=" << std::setw(7) << j6_current_deg << "°"
                          << " J7_current=" << std::setw(7) << j7_current_deg << "°";
                
                if (has_planned)
                {
                    std::cerr << " | J6_planned=" << std::setw(7) << j6_planned_deg << "°"
                              << " J7_planned=" << std::setw(7) << j7_planned_deg << "°";
                }
                
                std::cerr << " | joint67Cost=" << std::setw(10) << cost
                          << " | (insideCost=" << insideCost_ << " outsideCost=" << outsideCost_ << ")"
                          << std::endl;
            }
        }
        
        return cost;
    }

    ScalarFunctionQuadraticApproximation Joint67WorkspaceCost::getQuadraticApproximation(
        scalar_t time, const vector_t& state,
        const TargetTrajectories& /* targetTrajectories */,
        const PreComputation& /* preComp */) const
    {
        ScalarFunctionQuadraticApproximation approximation;
        approximation.f = getValue(time, state, TargetTrajectories(), PreComputation());
        
        const size_t stateDim = state.size();
        approximation.dfdx = vector_t::Zero(stateDim);
        approximation.dfdxx = matrix_t::Zero(stateDim, stateDim);
        
        // For a 0-1 step function, the gradient is zero everywhere except at the boundary
        // where it's discontinuous. We return zero gradient and hessian, which is a
        // reasonable approximation for optimization purposes.
        // The optimizer will still see the cost difference and work to stay inside.
        
        return approximation;
    }
} // namespace ocs2::mobile_manipulator

