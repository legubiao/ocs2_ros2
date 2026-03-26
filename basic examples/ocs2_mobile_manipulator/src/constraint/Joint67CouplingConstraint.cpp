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
 
#include "ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h"
#include <cmath>
 
 namespace ocs2::mobile_manipulator
 {
     namespace
     {
         constexpr scalar_t kDegToRad = M_PI / 180.0;
         constexpr scalar_t kJ7Knee = 40.0 * kDegToRad;
         constexpr scalar_t kJ7Max = 90.0 * kDegToRad;
         constexpr scalar_t kJ6AtKnee = 60.0 * kDegToRad;
         constexpr scalar_t kJ6AtMax = 20.0 * kDegToRad;
         constexpr scalar_t kSlope = (kJ6AtMax - kJ6AtKnee) / (kJ7Max - kJ7Knee); // -0.8
     }
 
    Joint67CouplingConstraint::Joint67CouplingConstraint(int stateDim, int armDim, scalar_t smoothAbsEps)
        : StateConstraint(ConstraintOrder::Linear),
           stateDim_(stateDim),
           armDim_(armDim),
           baseStateDim_(stateDim - armDim),
           numArms_(armDim >= 7 ? armDim / 7 : 0),
          smoothAbsEps_(smoothAbsEps)
     {
     }
 
     size_t Joint67CouplingConstraint::getNumConstraints(scalar_t /*time*/) const
     {
         return static_cast<size_t>(numArms_);
     }
 
     vector_t Joint67CouplingConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                                  const PreComputation& /*preComputation*/) const
     {
        vector_t value = vector_t::Zero(getNumConstraints(0.0));
         if (numArms_ == 0)
         {
             return value;
         }
 
         for (int arm = 0; arm < numArms_; ++arm)
         {
             const int armOffset = baseStateDim_ + arm * 7;
             const scalar_t q6 = state(armOffset + 5);
             const scalar_t q7 = state(armOffset + 6);
 
             const scalar_t absJ6 = smoothAbs(q6, smoothAbsEps_);
             const scalar_t absJ7 = smoothAbs(q7, smoothAbsEps_);
             const scalar_t limit = computeLimit(absJ7);

             // Output the margin h >= 0 for use with inequality penalties (e.g. relaxed barrier):
             //   h = limit(|q7|) - |q6|
             value(arm) = limit - absJ6;
         }

         return value;
     }
 
     VectorFunctionLinearApproximation Joint67CouplingConstraint::getLinearApproximation(
         scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComputation*/) const
     {
         VectorFunctionLinearApproximation approx(getNumConstraints(0.0), stateDim_);
         approx.f = getValue(0.0, state, PreComputation());
 
         if (numArms_ == 0)
         {
             return approx;
         }
 
         for (int arm = 0; arm < numArms_; ++arm)
         {
             const int armOffset = baseStateDim_ + arm * 7;
             const scalar_t q6 = state(armOffset + 5);
             const scalar_t q7 = state(armOffset + 6);
 
             const scalar_t absJ7 = smoothAbs(q7, smoothAbsEps_);
             const bool inKnee = (absJ7 <= kJ7Knee);
             const bool saturatedAtMax = (absJ7 >= kJ7Max);

             const scalar_t dabsJ6 = smoothAbsDerivative(q6, smoothAbsEps_);
             const scalar_t dabsJ7 = smoothAbsDerivative(q7, smoothAbsEps_);
             const scalar_t dlimit = (inKnee || saturatedAtMax) ? 0.0 : kSlope * dabsJ7;

             // h = limit - |q6|
             approx.dfdx(arm, armOffset + 5) = -dabsJ6;
             approx.dfdx(arm, armOffset + 6) = dlimit;
         }
 
         return approx;
     }
 
     scalar_t Joint67CouplingConstraint::smoothAbs(scalar_t x, scalar_t eps)
     {
         return std::sqrt(x * x + eps);
     }
 
     scalar_t Joint67CouplingConstraint::smoothAbsDerivative(scalar_t x, scalar_t eps)
     {
         return x / std::sqrt(x * x + eps);
     }

     scalar_t Joint67CouplingConstraint::computeLimit(scalar_t absJ7) const
     {
         if (absJ7 <= kJ7Knee)
         {
             return kJ6AtKnee;
         }
 
         const scalar_t limit = kJ6AtKnee + kSlope * (absJ7 - kJ7Knee);
         return std::max(limit, kJ6AtMax);
     }
 } // namespace ocs2::mobile_manipulator
