/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "packages/ros_bridge/components/RosNode.hpp"
#include "packages/ros_bridge/gems/pose_mapping.hpp"
#include "tf2_ros/transform_listener.h"

#include "engine/alice/alice_codelet.hpp"
#include "messages/rigid_body_3_group.capnp.h"


namespace isaac {
namespace ros_bridge {

// For a list of pose mappings, reads pose from ROS tf2 and writes it to Isaac Pose Tree.
// Note that frame definitions between Isaac and ROS may not match, e.g., "map" frame
// of Isaac and "map" frame of ROS are typically different.
class RosToRigid : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // Name of the Isaac node with RosNode component
  // Needs to be set before the application starts.
  ISAAC_PARAM(std::string, ros_node, "ros_node");
  ISAAC_PARAM(std::vector<IsaacRosPoseMapping>, pose_mappings, {});

  // Output group with a single body
  ISAAC_PROTO_TX(RigidBody3GroupProto, bodies);

  // Name of the body
  ISAAC_PARAM(std::string, body_name, "dummy_body");
  // Reference frame for the body
  ISAAC_PARAM(std::string, reference_frame, "world");
  // Pose of the body with respect to the reference frame
  ISAAC_PARAM(Pose3d, pose, Pose3d::Identity());
  // Linear velocity of the body
  ISAAC_PARAM(Vector3d, linear_velocity, Vector3d::Zero());
  // Angular velocity of the body
  ISAAC_PARAM(Vector3d, angular_velocity, Vector3d::Zero());
  // Linear acceleration of the body
  ISAAC_PARAM(Vector3d, linear_acceleration, Vector3d::Zero());
  // Angular acceleration of the body
  ISAAC_PARAM(Vector3d, angular_acceleration, Vector3d::Zero());




 private:
  RosNode* ros_node_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::RosToRigid);
