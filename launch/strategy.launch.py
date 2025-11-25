import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from launch_ros.actions import Node, SetParameter

def launch_setup(context, *args, **kwargs):
  eureka_bt = Node(
      package="eureka_bt",
      executable="eureka_bt",
      output="log",
    #   prefix="gdbserver :3000",
      parameters=[{
          "length_error_delta": -0.3,
          "buffer_size": 10,
          "navigation_time_limit": 60.0,
          "enough_close_to_republish": 5.0,
          "odometry_topic_name": "/odometry/filtered",
          "too_far_distance": 11.0,
          "allow_length_error": 2.0,
          "allow_angle_error": 3.0,
          "dummy_rotate_duration": 5.0,
          "too_big_angle": 9.0,
      }],
      remappings=[],
      arguments=["--ros-args", "--log-level", "info"]
  )

  return [SetParameter(name='use_sim_time', value=False),
          eureka_bt
  ]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
