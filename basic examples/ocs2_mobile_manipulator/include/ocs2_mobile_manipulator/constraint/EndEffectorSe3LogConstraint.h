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

#include <string>
#include <vector>

#include <Eigen/Core>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/spatial/se3.hpp>

namespace ocs2::mobile_manipulator
{
    /** End-effector tracking constraint using SE(3) log-map error. */
    class EndEffectorSe3LogConstraint final : public StateConstraint
    {
    public:
        using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;

        enum class InvariantType
        {
            Right = 0,
            Left = 1
        };

        EndEffectorSe3LogConstraint(std::vector<std::string> endEffectorFrames,
                                    ReferenceManager& referenceManager,
                                    bool isDualArmMode,
                                    InvariantType invariantType = InvariantType::Right);

        ~EndEffectorSe3LogConstraint() override = default;
        EndEffectorSe3LogConstraint* clone() const override { return new EndEffectorSe3LogConstraint(*this); }

        size_t getNumConstraints(scalar_t time) const override;

        vector_t getValue(scalar_t time, const vector_t& state,
                          const PreComputation& preComputation) const override;

        VectorFunctionLinearApproximation getLinearApproximation(
            scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;

    private:
        std::vector<std::string> endEffectorFrames_;
        mutable std::vector<size_t> frameIds_;
        ReferenceManager* referenceManagerPtr_ = nullptr;
        bool isDualArmMode_ = false;
        InvariantType invariantType_ = InvariantType::Right;
        // PinocchioInterface is provided via MobileManipulatorPreComputation at evaluation time.
    };
} // namespace ocs2::mobile_manipulator

