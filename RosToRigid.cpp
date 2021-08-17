

/*
Merged RigidBody3GroupGenerator.cpp & RosToPoses.cpp 

Subscribe ros message using RosToPoses, 
publish pose information as rigid format using RigidBody3GroupGenerator to unity(isaac sim)
*/


/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosToRigid.hpp"
#include "messages/math.hpp"
#include <memory>
#include <string>
#include <iostream>

namespace isaac {
namespace ros_bridge {

namespace {
// Number of bodies published by this codelet.
constexpr size_t kNumBodies = 1;
}  // namespace

void RosToRigid::start() {
  // Get RosNode pointer
  const std::string ros_node_name = get_ros_node();
  ros_node_ = node()->app()->getNodeComponentOrNull<RosNode>(ros_node_name);
  if (!ros_node_) {
    reportFailure("No RosNode component named '%s'", ros_node_name.c_str());
    return;
  }
  // Check with RosNode
  if (auto maybe_error = ros_node_->checkBeforeInterface()) {
    reportFailure(maybe_error->c_str());
    return;
  }

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  tickPeriodically();
}

void RosToRigid::tick() {
  // Check with RosNode
  ASSERT(ros_node_, "Logic error");
  if (auto maybe_error = ros_node_->checkBeforeInterface()) {
    reportFailure(maybe_error->c_str());
    return;
  }

  ASSERT(tf2_buffer_, "Logic error");
  ASSERT(tf2_listener_, "Logic error");

  try {
    // Get transformation from ROS
    const geometry_msgs::TransformStamped transformStamped = tf2_buffer_->lookupTransform(
        "map", "base_link", ros::Time(0));

    const geometry_msgs::Transform& transform = transformStamped.transform;
    const geometry_msgs::Quaternion& rotation = transform.rotation;
    const geometry_msgs::Vector3& translation = transform.translation;

    // Create Isaac pose
    const Pose3d pose{
        SO3d::FromQuaternion(Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)),
        Vector3d{translation.x, translation.z, translation.y}};
    
    auto group = tx_bodies().initProto();
    auto body = group.initBodies(kNumBodies)[0];
    ToProto(pose, body.initRefTBody());  // set pose(translation, rotation)
    ToProto(get_linear_velocity(), body.initLinearVelocity());
    ToProto(get_angular_velocity(), body.initAngularVelocity());
    ToProto(get_linear_acceleration(), body.initLinearAcceleration());
    ToProto(get_angular_acceleration(), body.initAngularAcceleration());
    ToProto(Vector3d::Constant(1.0), body.initScales());
    group.initNames(kNumBodies).set(0, get_body_name());
    group.setReferenceFrame(get_reference_frame());
    tx_bodies().publish();
  } 
  catch (...) {
    // tf2::TransformException will be thrown if transformation could not be read
  }
  
}

void RosToRigid::stop() {
  // Reset listener before buffer
  tf2_listener_ = nullptr;
  tf2_buffer_ = nullptr;
}

}  // namespace ros_bridge
}  // namespace isaac
