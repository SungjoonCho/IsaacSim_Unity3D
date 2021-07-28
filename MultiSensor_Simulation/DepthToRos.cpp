/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "DepthToRos.hpp"

#include <vector>

#include "messages/image.hpp"

namespace isaac {
namespace ros_bridge {

bool DepthToRos::protoToRos(const alice::ProtoRx<ImageProto>& rx_proto, const ros::Time& ros_time,
                            sensor_msgs::Image& ros_message) {
  // Read from Isaac message
  auto reader = rx_proto.getProto();
  const std::vector<SharedBuffer>& buffers = rx_proto.buffers();
  ImageConstView1f image;
  if (!FromProto(reader, buffers, image)) {
    reportFailure("Reading input image from proto buffer failed.");
    return false;
  }
  const size_t cols = image.cols();
  const size_t rows = image.rows();
  const size_t channels = reader.getChannels();

  // Populate ROS message
  ros_message.header.stamp = ros_time;
  ros_message.header.frame_id = get_frame_id();
  ros_message.width = cols;
  ros_message.height = rows;
  ros_message.step = cols * channels * 4;
  if (channels == 1) {
    ros_message.encoding = "32FC1";
  } else {
    reportFailure("Unexpected number of channels: %d", channels);
    return false;
  }
  unsigned const char* image_ptr = reinterpret_cast<unsigned const char*>(image.element_wise_begin());
  ros_message.data = std::vector<unsigned char>(image_ptr, image_ptr + ros_message.step * rows);
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
