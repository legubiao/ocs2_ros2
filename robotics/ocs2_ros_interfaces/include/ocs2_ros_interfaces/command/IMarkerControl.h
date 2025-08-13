#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <string>

namespace ocs2 {

    /**
     * Abstract interface for marker control operations.
     * This interface allows JoystickMarkerWrapper to operate on markers without direct coupling.
     */
    class IMarkerControl {
    public:
        // Enum definitions
        enum class Mode { SINGLE_ARM, DUAL_ARM };
        enum class ArmType { LEFT, RIGHT };
        
        // Position and orientation operations
        virtual void setSingleArmPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = 0;
        virtual void setDualArmPose(ArmType armType, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = 0;
        virtual std::pair<Eigen::Vector3d, Eigen::Quaterniond> getSingleArmPose() const = 0;
        virtual std::pair<Eigen::Vector3d, Eigen::Quaterniond> getDualArmPose(ArmType armType) const = 0;
        
        // Trajectory sending
        virtual void sendSingleArmTrajectories() = 0;
        virtual void sendDualArmTrajectories() = 0;
        
        // Mode control
        virtual void togglePublishMode() = 0;
        virtual bool isContinuousMode() const = 0;
        
        // Status queries
        virtual Mode getMode() const = 0;
        virtual ArmType getActiveArm() const = 0;
        virtual void setActiveArm(ArmType armType) = 0;
        
        // Display updates
        virtual void updateMarkerDisplay(const std::string& markerName, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = 0;
        
        virtual ~IMarkerControl() = default;
    };

} // namespace ocs2 