#include <pinocchio/fwd.hpp>

#include "ocs2_mobile_manipulator/collision/EnvironmentGeometryInterface.h"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <iostream>

namespace ocs2::mobile_manipulator {

EnvironmentGeometryInterface::EnvironmentGeometryInterface(
    const PinocchioGeometryInterface& pinocchioGeomInterface,
    const PinocchioInterface& pinocchioInterface,
    const std::vector<std::string>& robotCollisionLinks)
    : robotGeomModelPtr_(&pinocchioGeomInterface.getGeometryModel())
{
    filterRobotGeometries(pinocchioInterface, robotCollisionLinks);
}

void EnvironmentGeometryInterface::filterRobotGeometries(
    const PinocchioInterface& pinocchioInterface,
    const std::vector<std::string>& robotCollisionLinks)
{
    const auto& model = pinocchioInterface.getModel();

    if (robotCollisionLinks.empty()) {
        // Use all geometries from the existing geometry model
        for (size_t i = 0; i < robotGeomModelPtr_->geometryObjects.size(); ++i) {
            robotGeomIndices_.push_back(i);
        }
    } else {
        // Filter geometries by collision links
        for (size_t i = 0; i < robotGeomModelPtr_->geometryObjects.size(); ++i) {
            const auto& geomObj = robotGeomModelPtr_->geometryObjects[i];
            const std::string& frameName = model.frames[geomObj.parentFrame].name;

            for (const auto& linkName : robotCollisionLinks) {
                if (frameName == linkName || frameName.find(linkName) != std::string::npos) {
                    robotGeomIndices_.push_back(i);
                    break;
                }
            }
        }
    }

    std::cerr << "[EnvironmentGeometryInterface] Using " << robotGeomIndices_.size()
              << " robot collision geometries for environment collision checking.\n";

    // Print loaded geometries
    for (size_t idx : robotGeomIndices_) {
        const auto& geomObj = robotGeomModelPtr_->geometryObjects[idx];
        std::cerr << "  - " << geomObj.name << " (parent joint: "
                  << model.names[geomObj.parentJoint] << ")\n";
    }
}

void EnvironmentGeometryInterface::addBox(
    const std::string& name,
    const vector3_t& halfExtents,
    const vector3_t& position,
    const quaternion_t& orientation,
    double minimumDistance)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Obstacle obs;
    obs.name = name;
    obs.geometry = std::make_shared<ocs2::collision::Box>(
        halfExtents.x() * 2, halfExtents.y() * 2, halfExtents.z() * 2);
    obs.transform.setTranslation(position);
    obs.transform.setRotation(orientation.toRotationMatrix());
    obs.minimumDistance = minimumDistance;

    obstacles_[name] = std::move(obs);

    std::cerr << "[EnvironmentGeometryInterface] Added box obstacle: " << name
              << " at (" << position.transpose() << ")"
              << " half-extents: (" << halfExtents.transpose() << ")"
              << " minDist: " << minimumDistance << "\n";
}

void EnvironmentGeometryInterface::addSphere(
    const std::string& name,
    double radius,
    const vector3_t& position,
    double minimumDistance)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Obstacle obs;
    obs.name = name;
    obs.geometry = std::make_shared<ocs2::collision::Sphere>(radius);
    obs.transform.setTranslation(position);
    obs.transform.setRotation(Eigen::Matrix3d::Identity());
    obs.minimumDistance = minimumDistance;

    obstacles_[name] = std::move(obs);

    std::cerr << "[EnvironmentGeometryInterface] Added sphere obstacle: " << name
              << " at (" << position.transpose() << ")"
              << " radius: " << radius
              << " minDist: " << minimumDistance << "\n";
}

void EnvironmentGeometryInterface::addCylinder(
    const std::string& name,
    double radius, double height,
    const vector3_t& position,
    const quaternion_t& orientation,
    double minimumDistance)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Obstacle obs;
    obs.name = name;
    obs.geometry = std::make_shared<ocs2::collision::Cylinder>(radius, height);
    obs.transform.setTranslation(position);
    obs.transform.setRotation(orientation.toRotationMatrix());
    obs.minimumDistance = minimumDistance;

    obstacles_[name] = std::move(obs);

    std::cerr << "[EnvironmentGeometryInterface] Added cylinder obstacle: " << name
              << " at (" << position.transpose() << ")"
              << " radius: " << radius << " height: " << height
              << " minDist: " << minimumDistance << "\n";
}

void EnvironmentGeometryInterface::removeObstacle(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = obstacles_.find(name);
    if (it != obstacles_.end()) {
        obstacles_.erase(it);
        std::cerr << "[EnvironmentGeometryInterface] Removed obstacle: " << name << "\n";
    }
}

void EnvironmentGeometryInterface::updateObstaclePose(
    const std::string& name,
    const vector3_t& position,
    const quaternion_t& orientation)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = obstacles_.find(name);
    if (it != obstacles_.end()) {
        it->second.transform.setTranslation(position);
        it->second.transform.setRotation(orientation.toRotationMatrix());
    } else {
        std::cerr << "[EnvironmentGeometryInterface] Warning: obstacle not found: " << name << "\n";
    }
}

void EnvironmentGeometryInterface::clearAllObstacles() {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_.clear();
    std::cerr << "[EnvironmentGeometryInterface] Cleared all obstacles\n";
}

bool EnvironmentGeometryInterface::hasObstacle(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return obstacles_.find(name) != obstacles_.end();
}

std::vector<std::string> EnvironmentGeometryInterface::getObstacleNames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(obstacles_.size());
    for (const auto& [name, _] : obstacles_) {
        names.push_back(name);
    }
    return names;
}

const EnvironmentGeometryInterface::Obstacle* EnvironmentGeometryInterface::getObstacle(
    const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = obstacles_.find(name);
    if (it != obstacles_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<EnvironmentGeometryInterface::DistanceResultExt>
EnvironmentGeometryInterface::computeDistances(
    const PinocchioInterface& pinocchioInterface) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<DistanceResultExt> results;

    if (obstacles_.empty() || robotGeomIndices_.empty()) {
        return results;
    }

    // Create geometry data and update placements
    pinocchio::GeometryData robotGeomData(*robotGeomModelPtr_);
    pinocchio::updateGeometryPlacements(
        pinocchioInterface.getModel(),
        pinocchioInterface.getData(),
        *robotGeomModelPtr_,
        robotGeomData);

    // For each robot collision geometry and each obstacle, compute distance
    ocs2::collision::DistanceRequest request(true);  // Enable nearest points computation

    results.reserve(robotGeomIndices_.size() * obstacles_.size());

    for (size_t robotIdx : robotGeomIndices_) {
        const auto& robotGeomObj = robotGeomModelPtr_->geometryObjects[robotIdx];
        const auto& robotPlacement = robotGeomData.oMg[robotIdx];

        // Build robot collision object transform
        ocs2::collision_transform_t robotTransform;
        robotTransform.setTranslation(robotPlacement.translation());
        robotTransform.setRotation(robotPlacement.rotation());

        for (const auto& [obsName, obstacle] : obstacles_) {
            ocs2::collision::DistanceResult fclResult;

            ocs2::collision::distance(
                robotGeomObj.geometry.get(), robotTransform,
                obstacle.geometry.get(), obstacle.transform,
                request, fclResult);

            DistanceResultExt result;
            result.distance = fclResult.min_distance;
            result.nearestPointRobot = Eigen::Vector3d(
                fclResult.nearest_points[0][0],
                fclResult.nearest_points[0][1],
                fclResult.nearest_points[0][2]);
            result.nearestPointObstacle = Eigen::Vector3d(
                fclResult.nearest_points[1][0],
                fclResult.nearest_points[1][1],
                fclResult.nearest_points[1][2]);
            result.robotGeomIndex = robotIdx;
            result.obstacleName = obsName;
            result.robotJointIndex = robotGeomObj.parentJoint;
            result.minimumDistance = obstacle.minimumDistance;

            results.push_back(result);
        }
    }

    return results;
}

size_t EnvironmentGeometryInterface::getNumCollisionPairs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return robotGeomIndices_.size() * obstacles_.size();
}

size_t EnvironmentGeometryInterface::getNumObstacles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return obstacles_.size();
}

} // namespace ocs2::mobile_manipulator
