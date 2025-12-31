/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
    class GeometryInterfaceVisualization : public rclcpp::Node {
    public:
        /**
         * Constructor
         * @param pinocchioInterface: Pinocchio interface for the robot
         * @param geometryInterface: Geometry interface for collision checking
         * @param pinocchioWorldFrame: World frame name for visualization
         * @param activationDistance: Only show markers when distance < activationDistance.
         *                            If <= 0, show all markers (default behavior).
         */
        GeometryInterfaceVisualization(PinocchioInterface pinocchioInterface,
                                       PinocchioGeometryInterface geometryInterface,
                                       std::string pinocchioWorldFrame = "world",
                                       scalar_t activationDistance = -1.0);

        ~GeometryInterfaceVisualization() override = default;

        /**
         * Publish distance visualization markers
         * Only publishes markers for collision pairs where distance < activationDistance
         */
        void publishDistances(const vector_t &);

        /**
         * Set the activation distance for visualization filtering
         * @param activationDistance: Only show markers when distance < this value.
         *                            If <= 0, show all markers.
         */
        void setActivationDistance(scalar_t activationDistance) { activationDistance_ = activationDistance; }

        /**
         * Get the current activation distance
         */
        scalar_t getActivationDistance() const { return activationDistance_; }

        /**
         * Get the minimum distance from the last publishDistances() call
         * @return The minimum distance among all collision pairs, or max value if no pairs
         */
        scalar_t getLastMinDistance() const { return lastMinDistance_; }

        /**
         * Check if collision was detected in the last publishDistances() call
         * @param threshold: Distance threshold to consider as collision (default 0.0 = actual penetration)
         * @return true if any collision pair has distance <= threshold
         */
        bool isCollisionDetected(scalar_t threshold = 0.0) const { return lastMinDistance_ <= threshold; }

    private:
        PinocchioInterface pinocchioInterface_;
        PinocchioGeometryInterface geometryInterface_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markerPublisher_;

        std::string pinocchioWorldFrame_;
        scalar_t activationDistance_;  // Only show markers when distance < this value
        scalar_t lastMinDistance_{std::numeric_limits<scalar_t>::max()};  // Cached minimum distance from last update
    };
} // namespace ocs2
