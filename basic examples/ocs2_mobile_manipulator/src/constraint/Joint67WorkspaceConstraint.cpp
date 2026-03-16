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

#include "ocs2_mobile_manipulator/constraint/Joint67WorkspaceConstraint.h"
#include <algorithm>
#include <limits>

namespace ocs2::mobile_manipulator
{
    Joint67WorkspaceConstraint::Joint67WorkspaceConstraint(size_t j6Index, size_t j7Index)
        : StateConstraint(ConstraintOrder::Linear), j6Index_(j6Index), j7Index_(j7Index)
    {
    }

    std::vector<std::pair<scalar_t, scalar_t>> Joint67WorkspaceConstraint::getOctagonVertices()
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

    bool Joint67WorkspaceConstraint::isInsideWorkspace(scalar_t j6, scalar_t j7) const
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

    scalar_t Joint67WorkspaceConstraint::computeDistanceToBoundary(scalar_t j6, scalar_t j7) const
    {
        // For constraint: h >= 0 means inside workspace (satisfied)
        // h < 0 means outside workspace (violated)
        // We compute the minimum distance to the octagon boundary
        
        const auto vertices = getOctagonVertices();
        const size_t n = vertices.size();
        
        // If inside, return positive distance (margin to boundary)
        // If outside, return negative distance (violation amount)
        
        bool inside = isInsideWorkspace(j6, j7);
        
        if (inside) {
            // Compute minimum distance to any edge (positive = margin)
            scalar_t minDistance = std::numeric_limits<scalar_t>::max();
            
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const auto& vi = vertices[i];
                const auto& vj = vertices[j];
                
                // Vector from vi to vj
                const scalar_t dx = vj.first - vi.first;
                const scalar_t dy = vj.second - vi.second;
                const scalar_t edgeLen = std::sqrt(dx * dx + dy * dy);
                
                if (edgeLen < 1e-9) continue;
                
                // Vector from vi to point
                const scalar_t px = j6 - vi.first;
                const scalar_t py = j7 - vi.second;
                
                // Project point onto edge
                const scalar_t t = std::clamp((px * dx + py * dy) / (edgeLen * edgeLen), 0.0, 1.0);
                const scalar_t projX = vi.first + t * dx;
                const scalar_t projY = vi.second + t * dy;
                
                // Distance from point to projected point
                const scalar_t dist = std::sqrt((j6 - projX) * (j6 - projX) + (j7 - projY) * (j7 - projY));
                minDistance = std::min(minDistance, dist);
            }
            
            return minDistance;  // Positive = margin to boundary
        } else {
            // Outside: compute minimum distance to any edge (negative = violation)
            scalar_t minDistance = std::numeric_limits<scalar_t>::max();
            
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const auto& vi = vertices[i];
                const auto& vj = vertices[j];
                
                // Vector from vi to vj
                const scalar_t dx = vj.first - vi.first;
                const scalar_t dy = vj.second - vi.second;
                const scalar_t edgeLen = std::sqrt(dx * dx + dy * dy);
                
                if (edgeLen < 1e-9) continue;
                
                // Vector from vi to point
                const scalar_t px = j6 - vi.first;
                const scalar_t py = j7 - vi.second;
                
                // Project point onto edge
                const scalar_t t = std::clamp((px * dx + py * dy) / (edgeLen * edgeLen), 0.0, 1.0);
                const scalar_t projX = vi.first + t * dx;
                const scalar_t projY = vi.second + t * dy;
                
                // Distance from point to projected point
                const scalar_t dist = std::sqrt((j6 - projX) * (j6 - projX) + (j7 - projY) * (j7 - projY));
                minDistance = std::min(minDistance, dist);
            }
            
            return -minDistance;  // Negative = violation amount
        }
    }

    vector_t Joint67WorkspaceConstraint::getValue(scalar_t time, const vector_t& state,
                                                  const PreComputation& /* preComp */) const
    {
        if (j6Index_ >= state.size() || j7Index_ >= state.size())
        {
            // Return zero constraint (satisfied) if indices are invalid
            return vector_t::Zero(1);
        }
        
        scalar_t j6_current = state[j6Index_];
        scalar_t j7_current = state[j7Index_];
        
        // Constraint: h >= 0 means inside workspace (satisfied)
        // h < 0 means outside workspace (violated)
        scalar_t distance = computeDistanceToBoundary(j6_current, j7_current);
        
        vector_t constraint(1);
        constraint(0) = distance;  // Positive if inside, negative if outside
        
        return constraint;
    }

    VectorFunctionLinearApproximation Joint67WorkspaceConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& /* preComp */) const
    {
        VectorFunctionLinearApproximation approximation;
        approximation.f = getValue(time, state, PreComputation());
        
        const size_t stateDim = state.size();
        approximation.dfdx = vector_t::Zero(stateDim);
        
        // Compute gradient using finite difference
        const scalar_t eps = 1e-6;
        if (j6Index_ < stateDim && j7Index_ < stateDim) {
            // Gradient w.r.t. j6
            vector_t state_perturbed = state;
            state_perturbed(j6Index_) += eps;
            scalar_t h_perturbed = computeDistanceToBoundary(state_perturbed(j6Index_), state_perturbed(j7Index_));
            approximation.dfdx(j6Index_) = (h_perturbed - approximation.f(0)) / eps;
            
            // Gradient w.r.t. j7
            state_perturbed = state;
            state_perturbed(j7Index_) += eps;
            h_perturbed = computeDistanceToBoundary(state_perturbed(j6Index_), state_perturbed(j7Index_));
            approximation.dfdx(j7Index_) = (h_perturbed - approximation.f(0)) / eps;
        }
        
        return approximation;
    }
} // namespace ocs2::mobile_manipulator




