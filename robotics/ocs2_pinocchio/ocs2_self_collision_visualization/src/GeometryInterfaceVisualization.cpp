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

#include "ocs2_self_collision_visualization/GeometryInterfaceVisualization.h"

#include <iomanip>
#include <sstream>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {
    GeometryInterfaceVisualization::GeometryInterfaceVisualization(
        PinocchioInterface pinocchioInterface,
        PinocchioGeometryInterface geometryInterface,
        std::string pinocchioWorldFrame,
        scalar_t activationDistance)
        : Node("GeometryInterfaceVisualization"),
          pinocchioInterface_(std::move(pinocchioInterface)),
          geometryInterface_(std::move(geometryInterface)),
          markerPublisher_(
              this->create_publisher<visualization_msgs::msg::MarkerArray>(
                  "distance_markers", 1)),
          pinocchioWorldFrame_(std::move(pinocchioWorldFrame)),
          activationDistance_(activationDistance) {
    }


    void GeometryInterfaceVisualization::publishDistances(const vector_t &q) {
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        forwardKinematics(model, data, q);
        const auto results = geometryInterface_.computeDistances(pinocchioInterface_);

        // Update cached minimum distance
        lastMinDistance_ = std::numeric_limits<scalar_t>::max();
        for (const auto& result : results) {
            if (result.min_distance < lastMinDistance_) {
                lastMinDistance_ = result.min_distance;
            }
        }

        visualization_msgs::msg::MarkerArray markerArray;

        constexpr size_t numMarkersPerResult = 5;

        visualization_msgs::msg::Marker markerTemplate;
        markerTemplate.header.frame_id = pinocchioWorldFrame_;
        markerTemplate.header.stamp = rclcpp::Clock().now();
        markerTemplate.pose.orientation =
                ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});

        for (size_t i = 0; i < results.size(); ++i) {
            const scalar_t distance = results[i].min_distance;
            
            const std::string ns = std::to_string(
                geometryInterface_.getGeometryModel().collisionPairs[i].first) +
                " - " + std::to_string(
                geometryInterface_.getGeometryModel().collisionPairs[i].second);

            // Check if this collision pair should be hidden (distance >= activationDistance)
            const bool shouldHide = (activationDistance_ > 0.0 && distance >= activationDistance_);

            if (shouldHide) {
                // Publish DELETE markers to remove previously displayed markers for this collision pair
                for (size_t j = 0; j < numMarkersPerResult; ++j) {
                    visualization_msgs::msg::Marker deleteMarker;
                    deleteMarker.header.frame_id = pinocchioWorldFrame_;
                    deleteMarker.header.stamp = rclcpp::Clock().now();
                    deleteMarker.ns = ns;
                    deleteMarker.id = j;
                    deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
                    markerArray.markers.push_back(deleteMarker);
                }
                continue;
            }

            // Determine color based on distance (green = safe, yellow = warning, red = danger)
            std::array<scalar_t, 3> color;
            if (activationDistance_ > 0.0) {
                // Interpolate color based on distance relative to activation distance
                const scalar_t ratio = distance / activationDistance_;
                if (ratio > 0.5) {
                    // Green to yellow (safe to warning)
                    color = {2.0 * (1.0 - ratio), 1.0, 0.0};
                } else {
                    // Yellow to red (warning to danger)
                    color = {1.0, 2.0 * ratio, 0.0};
                }
            } else {
                // Default green color when no activation distance
                color = {0.0, 1.0, 0.0};
            }

            // The actual distance line, also denoting direction of the distance
            visualization_msgs::msg::Marker arrowMarker = markerTemplate;
            arrowMarker.action = visualization_msgs::msg::Marker::ADD;
            arrowMarker.ns = ns;
            arrowMarker.id = 0;
            arrowMarker.type = visualization_msgs::msg::Marker::ARROW;
            arrowMarker.color = ros_msg_helpers::getColor(color, 1.0);
            arrowMarker.points.push_back(
                ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
            arrowMarker.points.push_back(
                ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
            arrowMarker.scale.x = 0.01;
            arrowMarker.scale.y = 0.02;
            arrowMarker.scale.z = 0.04;
            markerArray.markers.push_back(arrowMarker);

            // Dots at the end of the arrow, denoting the close locations on the body
            visualization_msgs::msg::Marker sphereMarker = markerTemplate;
            sphereMarker.action = visualization_msgs::msg::Marker::ADD;
            sphereMarker.ns = ns;
            sphereMarker.id = 1;
            sphereMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            sphereMarker.color = ros_msg_helpers::getColor(color, 1.0);
            sphereMarker.points.push_back(
                ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
            sphereMarker.points.push_back(
                ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
            sphereMarker.scale.x = 0.02;
            sphereMarker.scale.y = 0.02;
            sphereMarker.scale.z = 0.02;
            markerArray.markers.push_back(sphereMarker);

            // Text denoting the object number for first point
            visualization_msgs::msg::Marker text1Marker = markerTemplate;
            text1Marker.action = visualization_msgs::msg::Marker::ADD;
            text1Marker.ns = ns;
            text1Marker.id = 2;
            text1Marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text1Marker.color = ros_msg_helpers::getColor(color, 1.0);
            text1Marker.scale.z = 0.02;
            text1Marker.pose.position =
                    ros_msg_helpers::getPointMsg(results[i].nearest_points[0]);
            text1Marker.pose.position.z += 0.015;
            text1Marker.text = "obj:" + std::to_string(
                geometryInterface_.getGeometryModel().collisionPairs[i].first);
            markerArray.markers.push_back(text1Marker);

            // Text denoting the object number for second point
            visualization_msgs::msg::Marker text2Marker = markerTemplate;
            text2Marker.action = visualization_msgs::msg::Marker::ADD;
            text2Marker.ns = ns;
            text2Marker.id = 3;
            text2Marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text2Marker.color = ros_msg_helpers::getColor(color, 1.0);
            text2Marker.scale.z = 0.02;
            text2Marker.pose.position =
                    ros_msg_helpers::getPointMsg(results[i].nearest_points[1]);
            text2Marker.pose.position.z += 0.015;
            text2Marker.text = "obj:" + std::to_string(
                geometryInterface_.getGeometryModel().collisionPairs[i].second);
            markerArray.markers.push_back(text2Marker);

            // Text above the arrow, denoting the distance
            visualization_msgs::msg::Marker distTextMarker = markerTemplate;
            distTextMarker.action = visualization_msgs::msg::Marker::ADD;
            distTextMarker.ns = ns;
            distTextMarker.id = 4;
            distTextMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            distTextMarker.color = ros_msg_helpers::getColor(color, 1.0);
            distTextMarker.scale.z = 0.02;
            distTextMarker.pose.position = ros_msg_helpers::getPointMsg(
                (results[i].nearest_points[0] + results[i].nearest_points[1]) / 2.0);
            distTextMarker.pose.position.z += 0.015;
            
            // Format distance with 3 decimal places
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << distance;
            distTextMarker.text = "dist:" +
                std::to_string(geometryInterface_.getGeometryModel().collisionPairs[i].first) +
                "-" +
                std::to_string(geometryInterface_.getGeometryModel().collisionPairs[i].second) +
                ":" + oss.str();
            markerArray.markers.push_back(distTextMarker);
        }

        markerPublisher_->publish(markerArray);
    }
} // namespace ocs2
