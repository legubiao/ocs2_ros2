/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_quadrotor_ros/QuadrotorDummyVisualization.h"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ocs2::quadrotor {

void QuadrotorDummyVisualization::update(const SystemObservation& observation,
                                         const PrimalSolution& policy,
                                         const CommandData& command) {
  const auto& targetTrajectories = command.mpcTargetTrajectories_;
  geometry_msgs::msg::TransformStamped command_frame_transform;
  command_frame_transform.header.stamp = node_->get_clock()->now();
  command_frame_transform.header.frame_id = "odom";
  command_frame_transform.child_frame_id = "command";

  command_frame_transform.transform.translation.x =
      targetTrajectories.stateTrajectory.back()(0);
  command_frame_transform.transform.translation.y =
      targetTrajectories.stateTrajectory.back()(1);
  command_frame_transform.transform.translation.z =
      targetTrajectories.stateTrajectory.back()(2);

  tf2::Quaternion desiredQuaternionBaseToWorld;
  desiredQuaternionBaseToWorld.setRPY(
      targetTrajectories.stateTrajectory.back()(3),
      targetTrajectories.stateTrajectory.back()(4),
      targetTrajectories.stateTrajectory.back()(5));

  command_frame_transform.transform.rotation.x =
      desiredQuaternionBaseToWorld.x();
  command_frame_transform.transform.rotation.y =
      desiredQuaternionBaseToWorld.y();
  command_frame_transform.transform.rotation.z =
      desiredQuaternionBaseToWorld.z();
  command_frame_transform.transform.rotation.w =
      desiredQuaternionBaseToWorld.w();
  tfBroadcaster_.sendTransform(command_frame_transform);

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = node_->get_clock()->now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "base";

  transformStamped.transform.translation.x = observation.state(0);
  transformStamped.transform.translation.y = observation.state(1);
  transformStamped.transform.translation.z = observation.state(2);

  tf2::Quaternion q;
  q.setRPY(observation.state(3), observation.state(4), observation.state(5));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  tfBroadcaster_.sendTransform(transformStamped);
}

}  // namespace ocs2::quadrotor
