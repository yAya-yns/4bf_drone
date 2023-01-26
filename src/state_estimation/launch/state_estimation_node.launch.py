import os
import sys
import yaml 
import launch
import launch.actions
import launch.events

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import launch_ros.actions
import launch_ros.events


def generate_launch_description():

  ld = LaunchDescription()
  
  config = os.path.join(
      get_package_share_directory('state_estimation'),
      'config',
      'config.yaml'
  )

  transform_path = os.path.join(
       get_package_share_directory('state_estimation'),
      'config',
      'transforms.yaml')
  transform_file = open(transform_path, 'r')
  transforms = yaml.safe_load(transform_file)["transforms"]

  state_estimation = Node(
      package="state_estimation",
      executable="state_estimation",
      name="state_estimation",
      output="screen",
      emulate_tty=True,
      parameters = [config]
  )

  camera_transform_args = \
      [str(transforms["camera_baselink_transform"]["x"]),
       str(transforms["camera_baselink_transform"]["y"]),
       str(transforms["camera_baselink_transform"]["z"]),
       str(transforms["camera_baselink_transform"]["r"]),
       str(transforms["camera_baselink_transform"]["p"]),
       str(transforms["camera_baselink_transform"]["y"])]

  camera_baselink_transform = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      arguments=camera_transform_args
  )

  lidar_transform_args = \
      [str(transforms["lidar_baselink_transform"]["x"]),
       str(transforms["lidar_baselink_transform"]["y"]),
       str(transforms["lidar_baselink_transform"]["z"]),
       str(transforms["lidar_baselink_transform"]["r"]),
       str(transforms["lidar_baselink_transform"]["p"]),
       str(transforms["lidar_baselink_transform"]["y"])]

  lidar_baselink_transform = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      arguments=lidar_transform_args
  )

  imu_transform_args = \
      [str(transforms["imu_baselink_transform"]["x"]),
       str(transforms["imu_baselink_transform"]["y"]),
       str(transforms["imu_baselink_transform"]["z"]),
       str(transforms["imu_baselink_transform"]["r"]),
       str(transforms["imu_baselink_transform"]["p"]),
       str(transforms["imu_baselink_transform"]["y"])]

  imu_baselink_transform = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      arguments=imu_transform_args
  )

  ld.add_action(state_estimation)
  ld.add_action(camera_baselink_transform)
  ld.add_action(lidar_baselink_transform)
  ld.add_action(imu_baselink_transform) 
  return ld