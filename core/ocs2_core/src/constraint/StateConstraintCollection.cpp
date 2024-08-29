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

#include <ocs2_core/constraint/StateConstraintCollection.h>

namespace ocs2 {
    StateConstraintCollection::StateConstraintCollection(const StateConstraintCollection &other) = default;


    StateConstraintCollection *StateConstraintCollection::clone() const {
        return new StateConstraintCollection(*this);
    }


    size_t StateConstraintCollection::getNumConstraints(scalar_t time) const {
        size_t numConstraints = 0;
        for (const auto &constraintTerm: this->terms_) {
            if (constraintTerm->isActive(time)) {
                numConstraints += constraintTerm->getNumConstraints(time);
            }
        }
        return numConstraints;
    }


    size_array_t StateConstraintCollection::getTermsSize(scalar_t time) const {
        size_array_t termsSize(this->terms_.size(), 0);
        for (size_t i = 0; i < this->terms_.size(); ++i) {
            if (this->terms_[i]->isActive(time)) {
                termsSize[i] = this->terms_[i]->getNumConstraints(time);
            }
        }
        return termsSize;
    }


    vector_array_t StateConstraintCollection::getValue(scalar_t time, const vector_t &state,
                                                       const PreComputation &preComp) const {
        vector_array_t constraintValues(this->terms_.size());
        for (size_t i = 0; i < this->terms_.size(); ++i) {
            if (this->terms_[i]->isActive(time)) {
                constraintValues[i] = this->terms_[i]->getValue(time, state, preComp);
            }
        } // end of i loop
        return constraintValues;
    }


    VectorFunctionLinearApproximation StateConstraintCollection::getLinearApproximation(
        scalar_t time, const vector_t &state,
        const PreComputation &preComp) const {
        VectorFunctionLinearApproximation linearApproximation(static_cast<int>(getNumConstraints(time)),
                                                              static_cast<int>(state.rows()));

        // append linearApproximation of each constraintTerm
        size_t i = 0;
        for (const auto &constraintTerm: this->terms_) {
            if (constraintTerm->isActive(time)) {
                const auto constraintTermApproximation = constraintTerm->getLinearApproximation(time, state, preComp);
                const size_t nc = constraintTermApproximation.f.rows();
                linearApproximation.f.segment(static_cast<long>(i), nc) = constraintTermApproximation.f;
                linearApproximation.dfdx.middleRows(static_cast<long>(i), nc) = constraintTermApproximation.dfdx;
                i += nc;
            }
        }

        return linearApproximation;
    }


    VectorFunctionQuadraticApproximation StateConstraintCollection::getQuadraticApproximation(
        scalar_t time, const vector_t &state,
        const PreComputation &preComp) const {
        const auto numConstraints = getNumConstraints(time);

        VectorFunctionQuadraticApproximation quadraticApproximation;
        quadraticApproximation.f.resize(static_cast<long>(numConstraints));
        quadraticApproximation.dfdx.resize(static_cast<long>(numConstraints), state.rows());
        quadraticApproximation.dfdxx.reserve(numConstraints);
        // Use reserve instead of resize to avoid unnecessary allocations.

        // append quadraticApproximation of each constraintTerm
        long i = 0;
        for (const auto &constraintTerm: this->terms_) {
            if (constraintTerm->isActive(time)) {
                auto constraintTermApproximation = constraintTerm->getQuadraticApproximation(time, state, preComp);
                const size_t nc = constraintTermApproximation.f.rows();
                quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
                quadraticApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
                appendVectorToVectorByMoving(quadraticApproximation.dfdxx,
                                             std::move(constraintTermApproximation.dfdxx));
                i += static_cast<long>(nc);
            }
        }

        return quadraticApproximation;
    }
} // namespace ocs2