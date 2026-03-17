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
 
 #include "ocs2_mobile_manipulator/LogQuadraticPenalty.h"
 #include <algorithm>
 #include <cmath>
 
 namespace ocs2::mobile_manipulator
 {
     LogQuadraticPenalty::LogQuadraticPenalty(scalar_t mu, scalar_t scale)
         : mu_(mu),
           scale_(std::max(scale, scalar_t(1e-12)))
     {
     }
 
     scalar_t LogQuadraticPenalty::getValue(scalar_t /*t*/, scalar_t h) const
     {
         const scalar_t ratio = (h * h) / scale_;
         return 0.5 * mu_ * std::log1p(ratio);
     }
 
     scalar_t LogQuadraticPenalty::getDerivative(scalar_t /*t*/, scalar_t h) const
     {
         const scalar_t denom = scale_ + h * h;
         return mu_ * h / denom;
     }
 
     scalar_t LogQuadraticPenalty::getSecondDerivative(scalar_t /*t*/, scalar_t h) const
     {
         const scalar_t denom = scale_ + h * h;
         return mu_ * (scale_ - h * h) / (denom * denom);
     }
 } // namespace ocs2::mobile_manipulator
