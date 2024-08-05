---
title: Ros bags
description: Some useful commands when using ROS bags.
tags:
  - ros2
---

# Ros bags

## Basic usage

`ros2 bag record -a -o my_bag_name` to record all topics to a given database name.
`ros2 bag info my_bag_name` to view what's in it.
`ros2 bag play my_bag_name` to play back everything in the bag.

## Using MCAP instead of SQLite for better performance

We've had issues when using ROS bags on an Raspberry PI, where the recording has been lagging. The first few seconds are fine, but then a lot of data is not present, which makes the recording pretty useless.

Here's how you can use MCAP instead of SQLite (which is the default), to potentially avoid this problem:

- `sudo apt-get install ros-$ROS_DISTRO-rosbag2-storage-mcap`
- `ros2 bag record --all --storage mcap --output MY_RECORDING`
- `ros2 bag info --storage mcap MY_RECORDING`
- `ros2 bag play --storage mcap MY_RECORDING`
