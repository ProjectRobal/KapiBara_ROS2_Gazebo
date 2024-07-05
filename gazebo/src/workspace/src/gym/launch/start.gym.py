import os
import logging
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    
    gym = Node(
        package="gym",
        executable="main.py"
    )
    # Run the node
    return LaunchDescription([
        gym
    ])


