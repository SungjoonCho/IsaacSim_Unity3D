"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app")

isaac_app(
    name = "multisensor_unity3d",
    data = [
        "//packages/navsim/apps:navsim_navigation_subgraph"
    ],
    modules = [
	"rgbd_processing",
        "sight",
        "viewers",
	"ros_bridge",
	"behavior_tree"
    ],
    visibility = ["//visibility:public"],
)
