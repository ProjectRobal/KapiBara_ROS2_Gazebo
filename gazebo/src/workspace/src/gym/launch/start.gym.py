import os
import logging
from launch import LaunchDescription,actions,substitutions

from launch_ros.actions import Node


def generate_launch_description():
    
    
    logger = substitutions.LaunchConfiguration("log_level")
    
    gym = Node(
        package="gym",
        executable="main.py",
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', logger]
    )
    
    
    # Run the node
    return LaunchDescription([
        actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
      ),
        gym
    ])


