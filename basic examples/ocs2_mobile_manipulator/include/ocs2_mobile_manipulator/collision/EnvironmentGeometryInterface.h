#pragma once

#include <ocs2_pinocchio_interface/collision_compat.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <pinocchio/multibody/geometry.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ocs2::mobile_manipulator {

/**
 * @brief Environment Geometry Interface - Manages environment obstacles and computes
 *        collision distances between robot and obstacles using coal.
 *
 * This class provides a thread-safe interface for dynamically adding, removing, and
 * updating obstacle poses at runtime. It computes distances and nearest points between
 * robot collision geometries and environment obstacles.
 */
class EnvironmentGeometryInterface {
public:
    using vector3_t = Eigen::Vector3d;
    using quaternion_t = Eigen::Quaterniond;

    /**
     * @brief Obstacle representation
     */
    struct Obstacle {
        std::string name;
        std::shared_ptr<ocs2::collision::CollisionGeometry> geometry;
        ocs2::collision_transform_t transform;
        double minimumDistance = 0.0;  ///< Per-obstacle minimum distance (0 uses default)
    };

    /**
     * @brief Extended distance result containing nearest points and geometry info
     */
    struct DistanceResultExt {
        double distance;                      ///< Distance between robot and obstacle
        Eigen::Vector3d nearestPointRobot;    ///< Nearest point on robot (world frame)
        Eigen::Vector3d nearestPointObstacle; ///< Nearest point on obstacle (world frame)
        size_t robotGeomIndex;                ///< Robot geometry index
        std::string obstacleName;             ///< Obstacle name
        pinocchio::JointIndex robotJointIndex; ///< Robot joint index (for Jacobian)
        double minimumDistance;               ///< Per-obstacle minimum distance
    };

    /**
     * @brief Constructor - reuses existing PinocchioGeometryInterface
     * @param pinocchioGeomInterface Existing geometry interface with robot collision model
     * @param robotCollisionLinks List of robot link names to check for collision with environment
     *                            If empty, all geometries from pinocchioGeomInterface are used
     */
    EnvironmentGeometryInterface(const PinocchioGeometryInterface& pinocchioGeomInterface,
                                  const PinocchioInterface& pinocchioInterface,
                                  const std::vector<std::string>& robotCollisionLinks = {});

    ~EnvironmentGeometryInterface() = default;

    // ========== Obstacle Management ==========

    /**
     * @brief Add a box obstacle
     * @param name Unique name for the obstacle
     * @param halfExtents Half-extents of the box (x, y, z)
     * @param position Position in world frame
     * @param orientation Orientation quaternion (default: identity)
     * @param minimumDistance Per-obstacle minimum distance (0 uses default from constraint)
     */
    void addBox(const std::string& name,
                const vector3_t& halfExtents,
                const vector3_t& position,
                const quaternion_t& orientation = quaternion_t::Identity(),
                double minimumDistance = 0.0);

    /**
     * @brief Add a sphere obstacle
     * @param name Unique name for the obstacle
     * @param radius Sphere radius
     * @param position Position in world frame
     * @param minimumDistance Per-obstacle minimum distance (0 uses default from constraint)
     */
    void addSphere(const std::string& name,
                   double radius,
                   const vector3_t& position,
                   double minimumDistance = 0.0);

    /**
     * @brief Add a cylinder obstacle
     * @param name Unique name for the obstacle
     * @param radius Cylinder radius
     * @param height Cylinder height (total, not half)
     * @param position Position in world frame (center of cylinder)
     * @param orientation Orientation quaternion (default: identity, z-axis aligned)
     * @param minimumDistance Per-obstacle minimum distance (0 uses default from constraint)
     */
    void addCylinder(const std::string& name,
                     double radius, double height,
                     const vector3_t& position,
                     const quaternion_t& orientation = quaternion_t::Identity(),
                     double minimumDistance = 0.0);

    /**
     * @brief Remove an obstacle by name
     * @param name Obstacle name to remove
     */
    void removeObstacle(const std::string& name);

    /**
     * @brief Update the pose of an existing obstacle
     * @param name Obstacle name
     * @param position New position in world frame
     * @param orientation New orientation quaternion
     */
    void updateObstaclePose(const std::string& name,
                            const vector3_t& position,
                            const quaternion_t& orientation = quaternion_t::Identity());

    /**
     * @brief Clear all obstacles
     */
    void clearAllObstacles();

    /**
     * @brief Check if an obstacle exists
     * @param name Obstacle name
     * @return true if obstacle exists
     */
    bool hasObstacle(const std::string& name) const;

    /**
     * @brief Get list of all obstacle names
     * @return Vector of obstacle names
     */
    std::vector<std::string> getObstacleNames() const;

    /**
     * @brief Get obstacle information for visualization
     * @param name Obstacle name
     * @return Pointer to obstacle, nullptr if not found
     */
    const Obstacle* getObstacle(const std::string& name) const;

    // ========== Collision Detection ==========

    /**
     * @brief Compute distances between all robot-obstacle pairs
     * @param pinocchioInterface Robot interface (must have forwardKinematics() called)
     * @return Vector of distance results for each collision pair
     * @note Requires pinocchio::forwardKinematics() to be called beforehand
     */
    std::vector<DistanceResultExt> computeDistances(
        const PinocchioInterface& pinocchioInterface) const;

    /**
     * @brief Get the number of collision pairs
     * @return Number of (robot geometry, obstacle) pairs
     */
    size_t getNumCollisionPairs() const;

    /**
     * @brief Get the number of obstacles
     */
    size_t getNumObstacles() const;

    /**
     * @brief Get the robot geometry model
     * @return Reference to the pinocchio geometry model
     */
    const pinocchio::GeometryModel& getRobotGeometryModel() const {
        return *robotGeomModelPtr_;
    }

    /**
     * @brief Get the number of robot collision geometries being checked
     */
    size_t getNumRobotCollisionGeometries() const {
        return robotGeomIndices_.size();
    }

private:
    /**
     * @brief Filter robot geometries by collision link names
     */
    void filterRobotGeometries(const PinocchioInterface& pinocchioInterface,
                                const std::vector<std::string>& robotCollisionLinks);

    const pinocchio::GeometryModel* robotGeomModelPtr_;  ///< Pointer to existing geometry model (not owned)
    std::vector<size_t> robotGeomIndices_;  ///< Indices of robot geometries to check

    std::unordered_map<std::string, Obstacle> obstacles_;
    mutable std::mutex mutex_;  ///< Thread safety for obstacle management
};

} // namespace ocs2::mobile_manipulator
