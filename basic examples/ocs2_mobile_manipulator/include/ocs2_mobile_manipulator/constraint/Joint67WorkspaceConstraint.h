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

#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/Types.h>
#include <vector>
#include <utility>
#include <cmath>

namespace ocs2::mobile_manipulator
{
    /**
     * @brief State constraint for J6 and J7 joint workspace (hard constraint)
     * 
     * This constraint enforces that J6 and J7 joints must stay within
     * the octagonal workspace region defined for Marvin M3S/M6S CCS arms.
     * 
     * The constraint value is:
     * - h = 0 when inside workspace (satisfied)
     * - h < 0 when outside workspace (violated)
     * 
     * When used with RelaxedBarrierPenalty, this creates a hard constraint:
     * penalty becomes extremely large when h < 0 (outside workspace).
     * 
     * The octagonal workspace is defined by the following vertices (in radians):
     * 1. (-20°, 90°) -> (-0.349, 1.571)
     * 2. (20°, 90°) -> (0.349, 1.571)
     * 3. (60°, 49°) -> (1.047, 0.855)
     * 4. (60°, -49°) -> (1.047, -0.855)
     * 5. (20°, -90°) -> (0.349, -1.571)
     * 6. (-20°, -90°) -> (-0.349, -1.571)
     * 7. (-60°, -49°) -> (-1.047, -0.855)
     * 8. (-60°, 49°) -> (-1.047, 0.855)
     */
    class Joint67WorkspaceConstraint final : public StateConstraint
    {
    public:
        /**
         * @brief Constructor
         * @param j6Index Index of J6 joint in state vector
         * @param j7Index Index of J7 joint in state vector
         */
        explicit Joint67WorkspaceConstraint(size_t j6Index, size_t j7Index);

        Joint67WorkspaceConstraint* clone() const override
        {
            return new Joint67WorkspaceConstraint(*this);
        }

        size_t getNumConstraints(scalar_t time) const override
        {
            return 1;  // Single constraint: workspace boundary
        }

        vector_t getValue(scalar_t time, const vector_t& state, 
                         const PreComputation& /* preComp */) const override;

        VectorFunctionLinearApproximation getLinearApproximation(
            scalar_t time, const vector_t& state,
            const PreComputation& preComp) const override;

    private:
        /**
         * @brief Check if point (j6, j7) is inside the octagonal workspace
         * @param j6 J6 joint angle in radians
         * @param j7 J7 joint angle in radians
         * @return true if inside workspace, false otherwise
         */
        bool isInsideWorkspace(scalar_t j6, scalar_t j7) const;

        /**
         * @brief Compute distance to workspace boundary (negative if outside)
         * @param j6 J6 joint angle in radians
         * @param j7 J7 joint angle in radians
         * @return Distance to boundary (positive if inside, negative if outside)
         */
        scalar_t computeDistanceToBoundary(scalar_t j6, scalar_t j7) const;

        size_t j6Index_;      // Index of J6 joint in state vector
        size_t j7Index_;      // Index of J7 joint in state vector
        
        static constexpr scalar_t DEG_TO_RAD = M_PI / 180.0;
        static std::vector<std::pair<scalar_t, scalar_t>> getOctagonVertices();
    };
} // namespace ocs2::mobile_manipulator




