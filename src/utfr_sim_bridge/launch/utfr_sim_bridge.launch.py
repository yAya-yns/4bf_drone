import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  ld = LaunchDescription()
  config = os.path.join(
      get_package_share_directory('utfr_sim_bridge'),
      'config',
      'config.yaml'
  )

  node=Node(
      package="utfr_sim_bridge",
      executable="utfr_sim_bridge",
      name="utfr_sim_bridge",
      output="screen",
      emulate_tty=True,
      parameters = [config]
  )

  ld.add_action(node)
  return ld
