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

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_self_collision/SelfCollisionCppAd.h>

namespace ocs2 {

class SelfCollisionConstraintCppAd final : public StateConstraint {
 public:
  using update_pinocchio_interface_callback =
      std::function<void(const vector_t& state, PinocchioInterfaceTpl<scalar_t>& pinocchioInterface)>;

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] mapping: pinocchio mapping from pinocchio states to ocs2 states
   * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
   * @param [in] minimumDistance: minimum allowed distance between collision pairs
   * @param [in] modelName: name of the generated model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                 available.
   * @param [in] verbose: If true, print information. Otherwise, no information is printed.
   */
  SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping,
                               PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance,
                               const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                               bool verbose = true);

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] mapping: pinocchio mapping from pinocchio states to ocs2 states
   * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
   * @param [in] minimumDistance: minimum allowed distance between collision pairs
   * @param [in] updateCallback: In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
   *                             use this callback.
   * @param [in] modelName: name of the generated model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                 available.
   * @param [in] verbose: If true, print information. Otherwise, no information is printed.
   */
  SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping,
                               PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance,
                               update_pinocchio_interface_callback updateCallback, const std::string& modelName,
                               const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);
  ~SelfCollisionConstraintCppAd() override = default;
  SelfCollisionConstraintCppAd* clone() const override { return new SelfCollisionConstraintCppAd(*this); }

  size_t getNumConstraints(scalar_t time) const override;

  /** Get the self collision distance values */
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation&) const override;

  /** Get the self collision distance approximation */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const PreComputation&) const override;

 private:
  SelfCollisionConstraintCppAd(const SelfCollisionConstraintCppAd& rhs);

  mutable PinocchioInterface pinocchioInterface_;
  SelfCollisionCppAd selfCollision_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
  update_pinocchio_interface_callback updateCallback_ = nullptr;
};

}  // namespace ocs2
