#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_mobile_manipulator/collision/EnvironmentGeometryInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2::mobile_manipulator {

/**
 * @brief Visualizes environment obstacles and robot-obstacle distances in RViz
 */
class EnvironmentCollisionVisualization {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for publishing
     * @param envGeomInterface Environment geometry interface
     * @param pinocchioInterface Pinocchio interface for FK
     * @param worldFrame World frame name
     * @param activationDistance Only show distance markers when distance < this value
     */
    EnvironmentCollisionVisualization(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
        const PinocchioInterface& pinocchioInterface,
        const std::string& worldFrame = "world",
        scalar_t activationDistance = -1.0);

    ~EnvironmentCollisionVisualization() = default;

    /**
     * @brief Publish obstacle markers (call once or when obstacles change)
     */
    void publishObstacles(bool logPublication = true);

    /**
     * @brief Publish distance visualization markers for current robot state
     * @param state Current robot state
     */
    void publishDistances(const vector_t& state);

    /**
     * @brief Clear all obstacle markers
     */
    void clearObstacles();

    /**
     * @brief Set activation distance for distance marker filtering
     */
    void setActivationDistance(scalar_t distance) { activationDistance_ = distance; }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface_;
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstaclePublisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr distancePublisher_;
    rclcpp::TimerBase::SharedPtr obstacleRepublishTimer_;
    
    std::string worldFrame_;
    scalar_t activationDistance_;
    
    /**
     * @brief Create marker for box obstacle
     */
    visualization_msgs::msg::Marker createBoxMarker(
        const std::string& name,
        const ocs2::collision::Box& box,
        const ocs2::collision_transform_t& transform,
        int id);
    
    /**
     * @brief Create marker for sphere obstacle
     */
    visualization_msgs::msg::Marker createSphereMarker(
        const std::string& name,
        const ocs2::collision::Sphere& sphere,
        const ocs2::collision_transform_t& transform,
        int id);
    
    /**
     * @brief Create marker for cylinder obstacle
     */
    visualization_msgs::msg::Marker createCylinderMarker(
        const std::string& name,
        const ocs2::collision::Cylinder& cylinder,
        const ocs2::collision_transform_t& transform,
        int id);
};

} // namespace ocs2::mobile_manipulator
