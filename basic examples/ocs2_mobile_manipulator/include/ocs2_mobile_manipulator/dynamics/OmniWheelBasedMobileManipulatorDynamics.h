#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2::mobile_manipulator
{
    /**
    * Implementation of an omni-wheel (holonomic) mobile manipulator.
    *
    * The base is modeled as a planar holonomic platform. The state of the robot is:
    * (base x, base y, base yaw, arm joints).
    *
    * The robot is assumed to be velocity controlled with base commands expressed in the
    * body frame: forward velocity vx, lateral velocity vy, and yaw rate omega.
    * Suitable for omni-wheel / mecanum bases at the planning level.
    */
    class OmniWheelBasedMobileManipulatorDynamics final : public SystemDynamicsBaseAD
    {
    public:
        /**
         * Constructor
         *
         * @param [in] modelInfo : The manipulator information.
         * @param [in] modelName : name of the generate model library
         * @param [in] modelFolder : folder to save the model library files to
         * @param [in] recompileLibraries : If true, always compile the model library, else try to load existing library if available.
         * @param [in] verbose : Display information.
         */
        OmniWheelBasedMobileManipulatorDynamics(ManipulatorModelInfo modelInfo, const std::string& modelName,
                                                const std::string& modelFolder = "/tmp/ocs2",
                                                bool recompileLibraries = true, bool verbose = true);

        ~OmniWheelBasedMobileManipulatorDynamics() override = default;

        OmniWheelBasedMobileManipulatorDynamics* clone() const override
        {
            return new OmniWheelBasedMobileManipulatorDynamics(*this);
        }

    private:
        OmniWheelBasedMobileManipulatorDynamics(const OmniWheelBasedMobileManipulatorDynamics& rhs) = default;

        ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                  const ad_vector_t& /*parameters*/) const override;

        const ManipulatorModelInfo info_;
    };
}
