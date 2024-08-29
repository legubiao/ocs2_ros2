//
// Created by biao on 24-8-29.
//

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>

ocs2::GaussNewtonDDP_MPC::GaussNewtonDDP_MPC(const mpc::Settings &mpcSettings, ddp::Settings ddpSettings,
                                             const RolloutBase &rollout,
                                             const OptimalControlProblem &optimalControlProblem,
                                             const Initializer &initializer): MPC_BASE(mpcSettings) {
    switch (ddpSettings.algorithm_) {
        case ddp::Algorithm::SLQ:
            ddpPtr_ = std::make_unique<SLQ>(ddpSettings, rollout, optimalControlProblem, initializer);
            break;
        case ddp::Algorithm::ILQR:
            ddpPtr_ = std::make_unique<ILQR>(ddpSettings, rollout, optimalControlProblem, initializer);
            break;
        default:
            throw std::runtime_error("Undefined ddp::Algorithm type!");
    }
}

void ocs2::GaussNewtonDDP_MPC::calculateController(const scalar_t initTime, const vector_t &initState,
                                                   const scalar_t finalTime) {
    if (settings().coldStart_) {
        ddpPtr_->reset();
    }
    ddpPtr_->run(initTime, initState, finalTime);
}
