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
 
 namespace ocs2::mobile_manipulator
 {
     /**
      * 6/7 joint coupling constraint for 7-DOF arms.
     *
     * Coupling model (degrees):
     *  - |J7| <= 49 deg  =>  |J6| <= 60 deg
     *  - 49 < |J7| <= 90 =>  |J6| <= 60 + (20-60)/(90-49) * (|J7| - 49) ≈ 60 - 0.976*(|J7|-49)
     *                             so |J6| = 20 at |J7|=90
     *
     * This matches the Marvin M3S/M6S CCS 6/7 joint coupling chart.
     *
     * Output convention:
     *  - This constraint returns the inequality margin h = limit(|J7|) - |J6|.
     *  - Feasible region: h >= 0.
      */
     class Joint67CouplingConstraint final : public StateConstraint
     {
     public:
    Joint67CouplingConstraint(int stateDim, int armDim, scalar_t smoothAbsEps = 1e-6);
         ~Joint67CouplingConstraint() override = default;
 
         Joint67CouplingConstraint* clone() const override
         {
             return new Joint67CouplingConstraint(*this);
         }
 
         size_t getNumConstraints(scalar_t time) const override;
         vector_t getValue(scalar_t time, const vector_t& state,
                           const PreComputation& preComputation) const override;
         VectorFunctionLinearApproximation getLinearApproximation(
             scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
 
     private:
         static scalar_t smoothAbs(scalar_t x, scalar_t eps);
        static scalar_t smoothAbsDerivative(scalar_t x, scalar_t eps);
 
         scalar_t computeLimit(scalar_t absJ7) const;
 
         int stateDim_;
         int armDim_;
         int baseStateDim_;
         int numArms_;
        scalar_t smoothAbsEps_;
     };
 } // namespace ocs2::mobile_manipulator
