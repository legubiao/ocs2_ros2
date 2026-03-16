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

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/PreComputation.h>
#include <vector>
#include <array>
#include <cmath>
#include <utility>

namespace ocs2::mobile_manipulator
{
    /**
     * @brief State cost for J6 and J7 joint workspace constraint
     * 
     * This cost penalizes configurations where J6 and J7 joints are outside
     * the octagonal workspace region defined for Marvin M3S/M6S CCS arms.
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
    class Joint67WorkspaceCost : public ocs2::StateCost
    {
    public:
        /**
         * @brief Constructor
         * @param j6Index Index of J6 joint in state vector
         * @param j7Index Index of J7 joint in state vector
         * @param insideCost Cost when inside workspace (must be set in task.info)
         * @param outsideCost Cost when outside workspace (must be set in task.info)
         * @param enableLogging Enable cost logging (default: true)
         */
        Joint67WorkspaceCost(size_t j6Index, size_t j7Index, scalar_t insideCost, scalar_t outsideCost, bool enableLogging = true);

        Joint67WorkspaceCost* clone() const override;

        scalar_t getValue(scalar_t time, const vector_t& state, 
                         const TargetTrajectories& targetTrajectories,
                         const PreComputation& preComp) const override;

        ScalarFunctionQuadraticApproximation getQuadraticApproximation(
            scalar_t time, const vector_t& state,
            const TargetTrajectories& targetTrajectories,
            const PreComputation& preComp) const override;

    private:
        /**
         * @brief Check if point (j6, j7) is inside the octagonal workspace
         * @param j6 J6 joint angle in radians
         * @param j7 J7 joint angle in radians
         * @return true if inside workspace, false otherwise
         */
        bool isInsideWorkspace(scalar_t j6, scalar_t j7) const;

        size_t j6Index_;      // Index of J6 joint in state vector
        size_t j7Index_;      // Index of J7 joint in state vector
        scalar_t insideCost_; // Cost when inside workspace
        scalar_t outsideCost_; // Cost when outside workspace
        
        // Logging control
        bool enableLogging_; // Whether to enable logging
        mutable size_t logCounter_; // Counter to control logging frequency
        static constexpr size_t LOG_INTERVAL = 10; // Log every N calls

        // Octagon vertices (in radians)
        // Order: top-left, top-right, right-top, right-bottom, bottom-right, bottom-left, left-bottom, left-top
        static constexpr scalar_t DEG_TO_RAD = M_PI / 180.0;
        static std::vector<std::pair<scalar_t, scalar_t>> getOctagonVertices();
    };
} // namespace ocs2::mobile_manipulator

