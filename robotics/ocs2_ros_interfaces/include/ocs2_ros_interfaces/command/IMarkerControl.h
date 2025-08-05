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
        // 枚举定义
        enum class Mode { SINGLE_ARM, DUAL_ARM };
        enum class ArmType { LEFT, RIGHT };
        
        // 位置和姿态操作
        virtual void setSingleArmPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = 0;
        virtual void setDualArmPose(ArmType armType, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = 0;
        virtual std::pair<Eigen::Vector3d, Eigen::Quaterniond> getSingleArmPose() const = 0;
        virtual std::pair<Eigen::Vector3d, Eigen::Quaterniond> getDualArmPose(ArmType armType) const = 0;
        
        // 轨迹发送
        virtual void sendSingleArmTrajectories() = 0;
        virtual void sendDualArmTrajectories() = 0;
        
        // 模式控制
        virtual void togglePublishMode() = 0;
        virtual bool isContinuousMode() const = 0;
        
        // 状态查询
        virtual Mode getMode() const = 0;
        virtual ArmType getActiveArm() const = 0;
        virtual void setActiveArm(ArmType armType) = 0;
        
        // 显示更新
        virtual void updateMarkerDisplay(const std::string& markerName, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = 0;
        
        virtual ~IMarkerControl() = default;
    };

} // namespace ocs2 