/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include <memory>

#include "ocs2_mpcnet_core/control/MpcnetOnnxController.h"

#include <iostream>


namespace ocs2::mpcnet {
    void MpcnetOnnxController::loadPolicyModel(const std::string &policyFilePath) {
        policyFilePath_ = policyFilePath;

        // create session
        Ort::SessionOptions sessionOptions;
        sessionOptions.SetIntraOpNumThreads(1);
        sessionOptions.SetInterOpNumThreads(1);
        sessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvironmentPtr_, policyFilePath_.c_str(), sessionOptions);

        // get input and output info
        inputNames_.clear();
        outputNames_.clear();
        inputShapes_.clear();
        outputShapes_.clear();
        const Ort::AllocatorWithDefaultOptions allocator;

        for (int i = 0; i < sessionPtr_->GetInputCount(); i++) {
            auto input_name = sessionPtr_->GetInputNameAllocated(i, allocator);
            inputNameAllocatedStrings.push_back(std::move(input_name));
            inputNames_.push_back(inputNameAllocatedStrings.back().get());
            inputShapes_.push_back(sessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }
        for (int i = 0; i < sessionPtr_->GetOutputCount(); i++) {
            auto output_name = sessionPtr_->GetOutputNameAllocated(i, allocator);
            outputNameAllocatedStrings.push_back(std::move(output_name));
            outputNames_.push_back(outputNameAllocatedStrings.back().get());
            outputShapes_.push_back(sessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }
    }


    vector_t MpcnetOnnxController::computeInput(const scalar_t t, const vector_t &x) {
        if (sessionPtr_ == nullptr) {
            throw std::runtime_error(
                "[MpcnetOnnxController::computeInput] cannot compute input, since policy model is not loaded.");
        }

        // create input tensor object
        Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> observation =
                mpcnetDefinitionPtr_->getObservation(t, x, referenceManagerPtr_->getModeSchedule(),
                                                     referenceManagerPtr_->getTargetTrajectories())
                .cast<tensor_element_t>();
        const Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<Ort::Value> inputValues;
        inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
            memoryInfo, observation.data(), observation.size(),
            inputShapes_[0].data(), inputShapes_[0].size()));

        // run inference
        const Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 1,
                                                                outputNames_.data(), 1);

        // evaluate output tensor object
        const Eigen::Map<Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> > action(
            outputValues[0].GetTensorMutableData<tensor_element_t>(),
            outputShapes_[0][1], outputShapes_[0][0]);
        auto [fst, snd] = mpcnetDefinitionPtr_->getActionTransformation(
            t, x, referenceManagerPtr_->getModeSchedule(), referenceManagerPtr_->getTargetTrajectories());

        // transform action
        return fst * action.cast<scalar_t>() + snd;
    }
} // namespace ocs2::mpcnet
