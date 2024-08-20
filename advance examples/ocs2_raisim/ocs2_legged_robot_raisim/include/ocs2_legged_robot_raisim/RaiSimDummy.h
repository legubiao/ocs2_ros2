//
// Created by tlab-uav on 8/20/24.
//

#ifndef RAISIMDUMMY_H
#define RAISIMDUMMY_H

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/node.hpp>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "RaiSimConversions.h"
#include <ocs2_raisim_core/RaisimRollout.h>

#include "RaiSimVisualizer.h"

namespace ocs2::legged_robot {
    class RaiSimDummy final : public rclcpp::Node {
        void publishGridMap(const std::string &frameId) const;

        std::string taskFile_;
        std::string urdfFile_;
        std::string referenceFile_;
        std::string raisimFile_;
        std::string resourcePath_;

        std::shared_ptr<LeggedRobotRaisimConversions> conversions_;
        std::unique_ptr<raisim::HeightMap> terrainPtr_;

        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmapPublisher_;

    public:
        explicit RaiSimDummy(const std::string &robot_name);

        ~RaiSimDummy() override = default;

        std::shared_ptr<LeggedRobotInterface> interface_;
        std::shared_ptr<RaisimRollout> rollout_;
        std::shared_ptr<RaiSimVisualizer> visualizer_;
    };
}
#endif  // RAISIMDUMMY_H
