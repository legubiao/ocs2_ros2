#include <pinocchio/fwd.hpp>

#include "ocs2_mobile_manipulator_ros/EnvironmentCollisionVisualization.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <chrono>
#include <iomanip>
#include <sstream>

namespace ocs2::mobile_manipulator {

EnvironmentCollisionVisualization::EnvironmentCollisionVisualization(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
    const PinocchioInterface& pinocchioInterface,
    const std::string& worldFrame,
    scalar_t activationDistance)
    : node_(node),
      envGeomInterface_(std::move(envGeomInterface)),
      pinocchioInterfacePtr_(std::make_unique<PinocchioInterface>(pinocchioInterface)),
      worldFrame_(worldFrame),
      activationDistance_(activationDistance)
{
    // Use transient_local QoS for obstacles so late subscribers can receive them
    rclcpp::QoS obstacleQos(1);
    obstacleQos.transient_local();
    obstaclePublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "environment_obstacles", obstacleQos);
    distancePublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "environment_distances", 1);
    obstacleRepublishTimer_ = node_->create_wall_timer(
        std::chrono::seconds(1), [this]() { publishObstacles(false); });
}

void EnvironmentCollisionVisualization::publishObstacles(bool logPublication)
{
    if (!envGeomInterface_) return;

    visualization_msgs::msg::MarkerArray markerArray;
    
    // First, delete all previous markers
    visualization_msgs::msg::Marker deleteAll;
    deleteAll.header.frame_id = worldFrame_;
    deleteAll.header.stamp = node_->get_clock()->now();
    deleteAll.ns = "env_obstacles";
    deleteAll.id = 0;
    deleteAll.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(deleteAll);

    const auto obstacleNames = envGeomInterface_->getObstacleNames();
    int id = 1;  // Start from 1 to avoid conflict with deleteAll marker
    
    for (const auto& name : obstacleNames) {
        const auto* obstacle = envGeomInterface_->getObstacle(name);
        if (!obstacle) continue;
        
        const auto* geom = obstacle->geometry.get();
        
        // Determine geometry type and create appropriate marker
        if (const auto* box = dynamic_cast<const ocs2::collision::Box*>(geom)) {
            markerArray.markers.push_back(createBoxMarker(name, *box, obstacle->transform, id++));
        }
        else if (const auto* sphere = dynamic_cast<const ocs2::collision::Sphere*>(geom)) {
            markerArray.markers.push_back(createSphereMarker(name, *sphere, obstacle->transform, id++));
        }
        else if (const auto* cylinder = dynamic_cast<const ocs2::collision::Cylinder*>(geom)) {
            markerArray.markers.push_back(createCylinderMarker(name, *cylinder, obstacle->transform, id++));
        }
    }
    
    obstaclePublisher_->publish(markerArray);
    
    if (logPublication && !obstacleNames.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Published %zu environment obstacles", obstacleNames.size());
    }
}

void EnvironmentCollisionVisualization::publishDistances(const vector_t& state)
{
    if (!envGeomInterface_ || envGeomInterface_->getNumObstacles() == 0) return;

    // Update robot FK
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    pinocchio::forwardKinematics(model, data, state);

    // Compute distances
    const auto results = envGeomInterface_->computeDistances(*pinocchioInterfacePtr_);

    visualization_msgs::msg::MarkerArray markerArray;
    
    const auto timeStamp = node_->get_clock()->now();
    
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        const scalar_t distance = result.distance;
        
        // Skip if outside activation distance
        if (activationDistance_ > 0.0 && distance >= activationDistance_) {
            // Send delete markers
            for (int j = 0; j < 3; ++j) {
                visualization_msgs::msg::Marker deleteMarker;
                deleteMarker.header.frame_id = worldFrame_;
                deleteMarker.header.stamp = timeStamp;
                deleteMarker.ns = "env_dist_" + std::to_string(i);
                deleteMarker.id = j;
                deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
                markerArray.markers.push_back(deleteMarker);
            }
            continue;
        }
        
        // Color based on distance
        std::array<scalar_t, 3> color;
        if (activationDistance_ > 0.0) {
            const scalar_t ratio = distance / activationDistance_;
            if (ratio < 0.3) {
                color = {1.0, 0.0, 0.0};  // Red - danger
            } else if (ratio < 0.6) {
                color = {1.0, 0.5, 0.0};  // Orange - warning
            } else {
                color = {0.0, 1.0, 0.0};  // Green - safe
            }
        } else {
            color = {0.0, 0.5, 1.0};  // Blue - default
        }
        
        const std::string ns = "env_dist_" + std::to_string(i);
        
        // Sphere at robot nearest point
        visualization_msgs::msg::Marker sphereRobot;
        sphereRobot.header.frame_id = worldFrame_;
        sphereRobot.header.stamp = timeStamp;
        sphereRobot.ns = ns;
        sphereRobot.id = 0;
        sphereRobot.type = visualization_msgs::msg::Marker::SPHERE;
        sphereRobot.action = visualization_msgs::msg::Marker::ADD;
        sphereRobot.pose.position = ros_msg_helpers::getPointMsg(result.nearestPointRobot);
        sphereRobot.pose.orientation.w = 1.0;
        sphereRobot.scale.x = sphereRobot.scale.y = sphereRobot.scale.z = 0.02;
        sphereRobot.color = ros_msg_helpers::getColor(color, 1.0);
        markerArray.markers.push_back(sphereRobot);
        
        // Sphere at obstacle nearest point
        visualization_msgs::msg::Marker sphereObs;
        sphereObs.header.frame_id = worldFrame_;
        sphereObs.header.stamp = timeStamp;
        sphereObs.ns = ns;
        sphereObs.id = 1;
        sphereObs.type = visualization_msgs::msg::Marker::SPHERE;
        sphereObs.action = visualization_msgs::msg::Marker::ADD;
        sphereObs.pose.position = ros_msg_helpers::getPointMsg(result.nearestPointObstacle);
        sphereObs.pose.orientation.w = 1.0;
        sphereObs.scale.x = sphereObs.scale.y = sphereObs.scale.z = 0.02;
        sphereObs.color = ros_msg_helpers::getColor(color, 1.0);
        markerArray.markers.push_back(sphereObs);
        
        // Line connecting nearest points
        visualization_msgs::msg::Marker line;
        line.header.frame_id = worldFrame_;
        line.header.stamp = timeStamp;
        line.ns = ns;
        line.id = 2;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.005;
        line.color = ros_msg_helpers::getColor(color, 0.8);
        line.points.push_back(ros_msg_helpers::getPointMsg(result.nearestPointRobot));
        line.points.push_back(ros_msg_helpers::getPointMsg(result.nearestPointObstacle));
        markerArray.markers.push_back(line);
        
        // Text showing distance
        visualization_msgs::msg::Marker text;
        text.header.frame_id = worldFrame_;
        text.header.stamp = timeStamp;
        text.ns = ns;
        text.id = 3;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        Eigen::Vector3d midPoint = (result.nearestPointRobot + result.nearestPointObstacle) / 2.0;
        text.pose.position = ros_msg_helpers::getPointMsg(midPoint);
        text.pose.position.z += 0.02;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.015;
        text.color = ros_msg_helpers::getColor(color, 1.0);
        
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << distance << "m";
        text.text = oss.str();
        markerArray.markers.push_back(text);
    }
    
    distancePublisher_->publish(markerArray);
}

void EnvironmentCollisionVisualization::clearObstacles()
{
    visualization_msgs::msg::MarkerArray markerArray;
    
    visualization_msgs::msg::Marker deleteAll;
    deleteAll.header.frame_id = worldFrame_;
    deleteAll.header.stamp = node_->get_clock()->now();
    deleteAll.action = visualization_msgs::msg::Marker::DELETEALL;
    
    deleteAll.ns = "env_obstacles";
    markerArray.markers.push_back(deleteAll);
    
    deleteAll.ns = "env_distances";
    markerArray.markers.push_back(deleteAll);
    
    obstaclePublisher_->publish(markerArray);
    distancePublisher_->publish(markerArray);
}

visualization_msgs::msg::Marker EnvironmentCollisionVisualization::createBoxMarker(
    const std::string& name,
    const ocs2::collision::Box& box,
    const ocs2::collision_transform_t& transform,
    int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = worldFrame_;
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "env_obstacles";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = transform.getTranslation()[0];
    marker.pose.position.y = transform.getTranslation()[1];
    marker.pose.position.z = transform.getTranslation()[2];
    
    Eigen::Quaterniond q(transform.getRotation());
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    
    marker.scale.x = box.halfSide[0] * 2;
    marker.scale.y = box.halfSide[1] * 2;
    marker.scale.z = box.halfSide[2] * 2;
    
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.6;
    
    return marker;
}

visualization_msgs::msg::Marker EnvironmentCollisionVisualization::createSphereMarker(
    const std::string& name,
    const ocs2::collision::Sphere& sphere,
    const ocs2::collision_transform_t& transform,
    int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = worldFrame_;
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "env_obstacles";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = transform.getTranslation()[0];
    marker.pose.position.y = transform.getTranslation()[1];
    marker.pose.position.z = transform.getTranslation()[2];
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = marker.scale.y = marker.scale.z = sphere.radius * 2;
    
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.6;
    
    return marker;
}

visualization_msgs::msg::Marker EnvironmentCollisionVisualization::createCylinderMarker(
    const std::string& name,
    const ocs2::collision::Cylinder& cylinder,
    const ocs2::collision_transform_t& transform,
    int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = worldFrame_;
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "env_obstacles";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = transform.getTranslation()[0];
    marker.pose.position.y = transform.getTranslation()[1];
    marker.pose.position.z = transform.getTranslation()[2];
    
    Eigen::Quaterniond q(transform.getRotation());
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    
    marker.scale.x = marker.scale.y = cylinder.radius * 2;
    marker.scale.z = cylinder.halfLength * 2;
    
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.6;
    
    return marker;
}

} // namespace ocs2::mobile_manipulator
