#include <memory>

#include <ocs2_switched_model_interface/logic/DynamicsParametersSynchronizedModule.h>

namespace switched_model {
    DynamicsParametersSynchronizedModule::DynamicsParametersSynchronizedModule()
        : newDynamicsParameters_(std::make_unique<ComKinoSystemDynamicsParameters<scalar_t> >(
            activeDynamicsParameters_)) {
    }

    void DynamicsParametersSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime,
                                                            const vector_t &initState,
                                                            const ocs2::ReferenceManagerInterface &referenceManager) {
        auto lockedDynamicsParameterPtr = newDynamicsParameters_.lock();
        activeDynamicsParameters_ = *lockedDynamicsParameterPtr; // Copy external parameters to the active parameter set
    }
} // namespace switched_model
